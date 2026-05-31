// VanFanController.ino
// ---------------------------------------------------------------------------
// Campervan PWM fan controller for the Noctua NV-FH2 8-channel hub.
//
// Hardware: Adafruit ESP32-S3 Reverse TFT Feather (#5691)
//           Adafruit Sensirion SHT41 over STEMMA QT (#5776)
//           Noctua NV-FH2 8-channel PWM fan hub
//           12V -> 5V buck converter feeding the Feather's USB pin
//
// Wiring — NV-FH2 4-pin input cable to Feather:
//   Blue   (pin 4, PWM)   -> A0
//   Yellow (pin 3, Tacho)  -> A1  (optional, for RPM readout)
//   Black  (pin 1, GND)   -> GND
//   Red    (pin 2, +12V)  -> not connected (hub powered via SATA)
//
// Wiring — 12V to 5V buck converter to Feather:
//   Buck 5V out  -> USB pin
//   Buck GND     -> GND
//
// Output:   25 kHz PWM signal on A0 to the NV-FH2's 4-pin input (pin 4).
// Input:    Open-collector tacho from fan 1 (pin 3) on A1, optional.
//
// Modes:
//   OFF       fans off
//   MANUAL    top button +10%, bottom button -10%
//   AUTO      top +1 deg C target, bottom -1 deg C target.
//             Fans off until cabin temp exceeds (target + 1 deg C), then
//             ramp linearly to 100% over RAMP_RANGE_C degrees.
//
// Buttons (NORMAL screen state):
//   Middle short:   cycle mode OFF -> MANUAL -> AUTO -> OFF
//   Middle long:    enter brightness adjust mode (5-second window)
//   Top short:      +setpoint (target in AUTO, fan % in MANUAL)
//   Bottom short:   -setpoint
//   Bottom long:    toggle RPM overlay inside the fan bar
//
// Buttons (BRIGHTNESS screen state):
//   Top short:      brighter (resets the 5-second timer)
//   Bottom short:   dimmer (resets the 5-second timer); stepping to 0 sets
//                   the wake-restore target to BRIGHTNESS_FALLBACK_PCT (20%)
//   Bottom long:    jump straight to 0% from a visible brightness; the
//                   pre-zero value is captured as the wake-restore target
//   Middle short:   exit early back to NORMAL
//   5s timeout:     transition to SCREEN_OFF
//
// Buttons (SCREEN_OFF):
//   Any:            wake screen to NORMAL; the wake press is consumed
//                   so it doesn't also cycle mode / adjust setpoint.
//                   If brightness was 0 when sleeping, wake restores it
//                   to the captured non-zero value (or 20% fallback).
//
// Persistence (NVS via Preferences):
//   currentMode, targetTempC, manualPercent, brightnessPct, rpmOverlay are
//   saved on change (debounced 1 second) and restored at boot.
//
// Required libraries (Library Manager): Adafruit GFX, Adafruit ST7735+ST7789,
// Adafruit SHT4x, Adafruit BusIO, Adafruit Unified Sensor.
// Preferences is built into the ESP32 core.
//
// Board: "Adafruit Feather ESP32-S3 Reverse TFT" via ESP32 by Espressif v3.x.
// ---------------------------------------------------------------------------

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_SHT4x.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <Preferences.h>
#include <SPI.h>
#include <Wire.h>

// =========================== Pin definitions ===============================

#define PWM_PIN     A0    // PWM out to NV-FH2 pin 4 (PWM signal)
#define TACHO_PIN   A1    // Tacho in from fan 1 pin 3 (open-collector, optional)

// Three buttons. On the Reverse TFT Feather D0 is the BOOT button (active
// LOW, internal pull-up) and D1/D2 are active HIGH with external pull-downs.
// Default physical layout: D2 top, D1 middle, D0 bottom (USB-C upper right).
#define BTN_TOP_PIN     2
#define BTN_MIDDLE_PIN  1
#define BTN_BOTTOM_PIN  0
const bool BTN_TOP_ACTIVE_LOW    = false;
const bool BTN_MIDDLE_ACTIVE_LOW = false;
const bool BTN_BOTTOM_ACTIVE_LOW = true;

// =========================== PWM configuration =============================

const uint32_t FAN_PWM_FREQ       = 25000;  // 25 kHz Intel / Noctua spec
const uint8_t  FAN_PWM_RESOLUTION = 8;      // 8-bit (0-255)
const uint32_t BL_PWM_FREQ        = 1000;   // backlight PWM, anything inaudible
const uint8_t  BL_PWM_RESOLUTION  = 8;

// =========================== Behaviour tunables ============================

const float DEAD_ZONE_C        = 1.0;
const float RAMP_RANGE_C       = 5.0;

// Minimum non-zero PWM% at which the fans reliably spin from a stopped
// state. Any non-zero setting (MANUAL or AUTO) is clamped up to this value;
// 0% is always allowed and means "off". Set from your fan's datasheet, or
// measure empirically with the tacho once that's soldered. Approximate
// values for common Noctua 12V models:
//
//   NF-A12x25 PWM         ~20%
//   NF-A12x25 LS-PWM      ~12%
//   NF-A14 PWM            ~20%
//   NF-A4x10 5V PWM       ~30%
//   industrialPPC series  ~15-20%
//
// 20% is a safe default that will keep any common Noctua 12V fan spinning;
// drop it if your specific fan starts reliably at a lower duty.
const int FAN_MIN_RUNNING_PCT  = 20;
const float DEFAULT_TARGET_C   = 22.0;
const int   DEFAULT_MANUAL_PCT = 30;
const float TARGET_STEP_C      = 1.0;
const float MIN_TARGET_C       = 10.0;
const float MAX_TARGET_C       = 35.0;

// MANUAL mode fan-speed ladder: top/bottom buttons step through these
// values, but any value between 1 and FAN_MIN_RUNNING_PCT - 1 is skipped
// at runtime by the step helpers (see nextFanStep/prevFanStep). The
// fine-grained 1/2/3/5% entries are only reachable if FAN_MIN_RUNNING_PCT
// is set low enough to allow them.
const int FAN_STEPS[]      = { 0, 1, 2, 3, 5, 10, 20, 30, 40, 50,
                               60, 70, 80, 90, 100 };
const int FAN_STEPS_COUNT  = sizeof(FAN_STEPS) / sizeof(FAN_STEPS[0]);

// Backlight step ladder, expressed in percent. 0% is fully off (same effect
// as the screen-off timeout) but is persistent. Fine resolution at the low
// end so dim settings are usable at night without the screen disappearing
// in a single click. setBacklight() maps these linearly to a 0-255 PWM duty.
const int BRIGHTNESS_STEPS[]     = { 0, 1, 2, 3, 5, 10, 20, 30, 40, 50, 60,
                                     70, 80, 90, 100 };
const int BRIGHTNESS_STEPS_COUNT = sizeof(BRIGHTNESS_STEPS) / sizeof(BRIGHTNESS_STEPS[0]);
const int BRIGHTNESS_DEFAULT_PCT  = 100;

// Value to restore brightness to when waking from a manual step-down to 0%.
// A long-press-bottom jump from a visible brightness instead restores the
// pre-zero brightness; see lastNonZeroBrightness in the brightness handlers.
const int BRIGHTNESS_FALLBACK_PCT = 20;

// Noctua / Intel 4-wire spec: 2 tacho pulses per revolution.
const float          TACHO_PULSES_PER_REV = 2.0;
const unsigned long  TACHO_MEASURE_MS     = 1000;
const uint16_t       TACHO_HYSTERESIS_RPM = 30;  // ignore jitter under this

// =========================== Timing ========================================

const unsigned long SENSOR_READ_MS          = 2000;
const unsigned long DISPLAY_UPDATE_MS       = 250;
const unsigned long DEBOUNCE_MS             = 30;
const unsigned long LONG_PRESS_MS           = 1000;
const unsigned long BRIGHTNESS_TIMEOUT_MS   = 5000;
const unsigned long PREFS_DEBOUNCE_MS       = 1000;

// =========================== Display colours ===============================

#define RGB565(r, g, b) ((uint16_t)((((r) >> 3) << 11) | (((g) >> 2) << 5) | ((b) >> 3)))

const uint16_t COLOUR_BG       = 0x0000;
const uint16_t COLOUR_HEADER   = ST77XX_CYAN;
const uint16_t COLOUR_TEMP     = RGB565(245, 208,   0);
const uint16_t COLOUR_TARGET   = RGB565(255, 152,   0);
const uint16_t COLOUR_FAN      = RGB565( 76, 175,  80);
const uint16_t COLOUR_RH       = RGB565(224, 224, 224);
const uint16_t COLOUR_LABEL    = RGB565(112, 112, 112);
const uint16_t COLOUR_MUT      = RGB565( 80,  80,  80);
const uint16_t COLOUR_DIVIDER  = RGB565( 30,  30,  30);
const uint16_t COLOUR_BAR_BG   = RGB565( 26,  26,  26);
const uint16_t COLOUR_BAR_BRDR = RGB565( 40,  40,  40);
const uint16_t COLOUR_RPM_TEXT = RGB565(255, 255, 255);
const uint16_t COLOUR_ALERT    = ST77XX_RED;

// =========================== Layout constants ==============================

const int SCREEN_W = 240;
const int SCREEN_H = 135;

const int COL_LEFT_CENTRE_X  = 60;
const int COL_RIGHT_CENTRE_X = 180;
const int DIVIDER_X          = 120;
const int DIVIDER_Y_TOP      = 22;
const int DIVIDER_Y_BOTTOM   = 90;

const int HEADER_BASELINE_Y  = 14;
const int LABEL_TOP_Y        = 22;
const int BIG_BASELINE_Y     = 78;
const int LABEL_BOTTOM_Y     = 104;
const int BAR_Y              = 114;
const int BAR_H              = 18;     // taller now to host the RPM overlay
const int BOTTOM_BASELINE_Y  = 130;
const int RPM_BASELINE_Y     = BAR_Y + BAR_H - 5;

const int RH_X         = 6;
const int BAR_X        = 60;
const int BAR_W        = 174;
const int RIGHT_EDGE_X = 234;

// =========================== Globals =======================================

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_SHT4x  sht4;

// Offscreen buffer: drawing happens here, then we blit the whole thing to
// the TFT in a single drawRGBBitmap call. This is what kills the flicker.
GFXcanvas16     canvas(SCREEN_W, SCREEN_H);

Preferences     prefs;

enum Mode : uint8_t { MODE_OFF = 0, MODE_MANUAL = 1, MODE_AUTO = 2 };
const char* MODE_LABELS[] = { "OFF", "FAN SPEED", "AUTO" };

enum ScreenState : uint8_t { SCREEN_NORMAL, SCREEN_BRIGHTNESS, SCREEN_OFF };

// User-facing state.
Mode        currentMode    = MODE_OFF;
int         manualPercent  = DEFAULT_MANUAL_PCT;
float       targetTempC    = DEFAULT_TARGET_C;
int         brightnessPct  = BRIGHTNESS_DEFAULT_PCT;     // 1-100 percent
int         lastNonZeroBrightness = BRIGHTNESS_FALLBACK_PCT;  // restore target for waking from 0%
bool        rpmOverlay     = false;

// Sensed values.
float       currentTempC   = NAN;
float       currentRH      = NAN;
int         currentFanPct  = 0;
uint16_t    currentRPM     = 0;

// Internal state.
ScreenState screenState    = SCREEN_NORMAL;
bool        displayDirty   = true;
bool        sht4Ok         = false;
volatile uint32_t tachoPulseCount = 0;

unsigned long lastSensorReadMs    = 0;
unsigned long lastDisplayMs       = 0;
unsigned long lastTachoMeasureMs  = 0;
unsigned long brightnessEnteredMs = 0;
bool          prefsDirty          = false;
unsigned long prefsDirtyMs        = 0;

// =========================== Button state ==================================

struct Button {
  uint8_t pin;
  bool    activeLow;
  bool    rawActive;
  unsigned long lastChangeMs;
  bool    pressed;
  unsigned long pressStartMs;
  bool    longFired;
  bool    consumed;
};

Button btnTop    = { BTN_TOP_PIN,    BTN_TOP_ACTIVE_LOW,    false, 0, false, 0, false, false };
Button btnMiddle = { BTN_MIDDLE_PIN, BTN_MIDDLE_ACTIVE_LOW, false, 0, false, 0, false, false };
Button btnBottom = { BTN_BOTTOM_PIN, BTN_BOTTOM_ACTIVE_LOW, false, 0, false, 0, false, false };

typedef void (*BtnHandler)();

// =========================== Forward declarations ==========================

void pollButton(Button& b, unsigned long now, BtnHandler shortPress, BtnHandler longPress);
void onMiddleShort();
void onMiddleLong();
void onTopShort();
void onBottomShort();
void onBottomLong();
void readSensor();
void computeFan();
void applyFanPercent(int pct);
void IRAM_ATTR tachoISR();
void updateRPM(unsigned long now);
void setBacklight(int pct);
void enterBrightnessMode(unsigned long now);
void exitBrightnessMode();
void wakeScreen();
void sleepScreen();
void markDirty();
void markPrefsDirty();
void maybeFlushPrefs(unsigned long now);
void loadPrefs();
void savePrefs();
void updateDisplay();
static void drawNormalScreen();
static void drawBrightnessScreen();

// =========================== Setup =========================================

void setup() {
  Serial.begin(115200);

  // STEMMA QT / I2C power rail.
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // Backlight on a PWM channel so we can dim it. Default to full until
  // prefs are loaded a moment later.
  ledcAttach(TFT_BACKLITE, BL_PWM_FREQ, BL_PWM_RESOLUTION);
  setBacklight(BRIGHTNESS_DEFAULT_PCT);

  // Display.
  tft.init(135, 240);
  tft.setRotation(1);
  tft.fillScreen(COLOUR_BG);

  // SHT41.
  Wire.begin();
  sht4Ok = sht4.begin();
  if (sht4Ok) {
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
  } else {
    Serial.println("SHT41 not found");
  }

  // Buttons.
  pinMode(btnTop.pin,    btnTop.activeLow    ? INPUT_PULLUP : INPUT_PULLDOWN);
  pinMode(btnMiddle.pin, btnMiddle.activeLow ? INPUT_PULLUP : INPUT_PULLDOWN);
  pinMode(btnBottom.pin, btnBottom.activeLow ? INPUT_PULLUP : INPUT_PULLDOWN);

  // Fan PWM output.
  ledcAttach(PWM_PIN, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);

  // Tacho input. Internal pull-up handles the open-collector output of the
  // fan; transitions on RISING edge are counted in the ISR.
  pinMode(TACHO_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TACHO_PIN), tachoISR, RISING);

  // Restore last state from flash.
  loadPrefs();
  setBacklight(brightnessPct);
  computeFan();

  Serial.println("Van fan controller ready");
}

// =========================== Main loop =====================================

void loop() {
  unsigned long now = millis();

  pollButton(btnTop,    now, onTopShort,    nullptr);
  pollButton(btnMiddle, now, onMiddleShort, onMiddleLong);
  pollButton(btnBottom, now, onBottomShort, onBottomLong);

  // Brightness mode timeout: 5s after entry, regardless of activity.
  if (screenState == SCREEN_BRIGHTNESS &&
      (now - brightnessEnteredMs) >= BRIGHTNESS_TIMEOUT_MS) {
    sleepScreen();
  }

  // Sensor read + fan re-evaluation.
  if (now - lastSensorReadMs >= SENSOR_READ_MS) {
    lastSensorReadMs = now;
    if (sht4Ok) {
      readSensor();
    }
    computeFan();
    markDirty();
  }

  // Tacho measurement (1Hz).
  updateRPM(now);

  // Coalesced preferences write.
  maybeFlushPrefs(now);

  // Render only when something has changed; throttled to DISPLAY_UPDATE_MS.
  if (screenState != SCREEN_OFF
      && displayDirty
      && (now - lastDisplayMs >= DISPLAY_UPDATE_MS)) {
    lastDisplayMs = now;
    updateDisplay();
    displayDirty = false;
  }
}

// =========================== Button polling ================================

void pollButton(Button& b, unsigned long now, BtnHandler shortPress, BtnHandler longPress) {
  bool raw    = digitalRead(b.pin);
  bool active = b.activeLow ? (raw == LOW) : (raw == HIGH);

  if (active != b.rawActive) {
    b.lastChangeMs = now;
    b.rawActive    = active;
  }

  bool stable = (now - b.lastChangeMs) >= DEBOUNCE_MS;

  if (stable && active != b.pressed) {
    b.pressed = active;
    if (b.pressed) {
      // Press edge.
      b.pressStartMs = now;
      b.longFired    = false;
      if (screenState == SCREEN_OFF) {
        // Wake and consume this press.
        wakeScreen();
        b.consumed = true;
      } else {
        b.consumed = false;
      }
    } else {
      // Release edge: short-press fires here.
      if (!b.consumed && !b.longFired && shortPress != nullptr) {
        shortPress();
      }
    }
  }

  // Long-press fires while held, once.
  if (b.pressed && !b.consumed && !b.longFired
      && (now - b.pressStartMs) >= LONG_PRESS_MS
      && longPress != nullptr) {
    longPress();
    b.longFired = true;
  }
}

// =========================== Button handlers ===============================

// Walk the FAN_STEPS ladder. Strict inequality so out-of-table values
// (e.g. legacy saves from earlier firmware) snap onto the ladder. Values
// between 1 and FAN_MIN_RUNNING_PCT - 1 are skipped: the buttons can park
// you on 0 (off) or on a value at or above the minimum, never in between.
static int nextFanStep(int current) {
  for (int i = 0; i < FAN_STEPS_COUNT; i++) {
    int v = FAN_STEPS[i];
    if (v > current && (v == 0 || v >= FAN_MIN_RUNNING_PCT)) return v;
  }
  return current;
}

static int prevFanStep(int current) {
  for (int i = FAN_STEPS_COUNT - 1; i >= 0; i--) {
    int v = FAN_STEPS[i];
    if (v < current && (v == 0 || v >= FAN_MIN_RUNNING_PCT)) return v;
  }
  return current;
}

static int nextBrightnessStep(int current) {
  for (int i = 0; i < BRIGHTNESS_STEPS_COUNT; i++) {
    if (BRIGHTNESS_STEPS[i] > current) return BRIGHTNESS_STEPS[i];
  }
  return BRIGHTNESS_STEPS[BRIGHTNESS_STEPS_COUNT - 1];
}

static int prevBrightnessStep(int current) {
  for (int i = BRIGHTNESS_STEPS_COUNT - 1; i >= 0; i--) {
    if (BRIGHTNESS_STEPS[i] < current) return BRIGHTNESS_STEPS[i];
  }
  return BRIGHTNESS_STEPS[0];
}

void onMiddleShort() {
  if (screenState == SCREEN_BRIGHTNESS) {
    exitBrightnessMode();
    return;
  }
  // Cycle mode.
  currentMode = (Mode)((currentMode + 1) % 3);
  computeFan();
  markDirty();
  markPrefsDirty();
}

void onMiddleLong() {
  if (screenState == SCREEN_NORMAL) {
    enterBrightnessMode(millis());
  }
}

void onTopShort() {
  if (screenState == SCREEN_BRIGHTNESS) {
    brightnessPct = nextBrightnessStep(brightnessPct);
    setBacklight(brightnessPct);
    brightnessEnteredMs = millis();   // reset 5s timeout on activity
    markDirty();
    markPrefsDirty();
    return;
  }
  switch (currentMode) {
    case MODE_MANUAL:
      manualPercent = nextFanStep(manualPercent);
      computeFan();
      markDirty();
      markPrefsDirty();
      break;
    case MODE_AUTO:
      targetTempC = constrain(targetTempC + TARGET_STEP_C, MIN_TARGET_C, MAX_TARGET_C);
      computeFan();
      markDirty();
      markPrefsDirty();
      break;
    case MODE_OFF:
    default:
      break;
  }
}

void onBottomShort() {
  if (screenState == SCREEN_BRIGHTNESS) {
    brightnessPct = prevBrightnessStep(brightnessPct);
    // Stepping all the way down to 0 is a "manual blackout"; wake should
    // restore to the fallback, not whatever the previous long-press capture
    // happened to leave behind.
    if (brightnessPct == 0) lastNonZeroBrightness = BRIGHTNESS_FALLBACK_PCT;
    setBacklight(brightnessPct);
    brightnessEnteredMs = millis();   // reset 5s timeout on activity
    markDirty();
    markPrefsDirty();
    return;
  }
  switch (currentMode) {
    case MODE_MANUAL:
      manualPercent = prevFanStep(manualPercent);
      computeFan();
      markDirty();
      markPrefsDirty();
      break;
    case MODE_AUTO:
      targetTempC = constrain(targetTempC - TARGET_STEP_C, MIN_TARGET_C, MAX_TARGET_C);
      computeFan();
      markDirty();
      markPrefsDirty();
      break;
    case MODE_OFF:
    default:
      break;
  }
}

void onBottomLong() {
  if (screenState == SCREEN_NORMAL) {
    rpmOverlay = !rpmOverlay;
    markDirty();
    markPrefsDirty();
  } else if (screenState == SCREEN_BRIGHTNESS) {
    // Jump directly to 0% from a visible brightness. Capture the pre-zero
    // value so wakeScreen() can restore exactly what you had; stepping the
    // ladder down to 0 instead lands on BRIGHTNESS_FALLBACK_PCT, see
    // onBottomShort.
    if (brightnessPct > 0) lastNonZeroBrightness = brightnessPct;
    brightnessPct = 0;
    setBacklight(brightnessPct);
    brightnessEnteredMs = millis();   // reset 5s timeout on activity
    markDirty();
    markPrefsDirty();
  }
}

// =========================== Sensor ========================================

void readSensor() {
  sensors_event_t humEvt, tempEvt;
  if (sht4.getEvent(&humEvt, &tempEvt)) {
    currentTempC = tempEvt.temperature;
    currentRH    = humEvt.relative_humidity;
  }
}

// =========================== Fan compute / apply ===========================

void computeFan() {
  int pct = 0;
  switch (currentMode) {
    case MODE_OFF:
      pct = 0;
      break;
    case MODE_MANUAL:
      pct = manualPercent;
      break;
    case MODE_AUTO: {
      if (isnan(currentTempC)) {
        pct = 0;
        break;
      }
      float threshold = targetTempC + DEAD_ZONE_C;
      if (currentTempC <= threshold) {
        pct = 0;
      } else {
        float over   = currentTempC - threshold;
        float ramped = (over / RAMP_RANGE_C) * 100.0f;
        if (ramped < FAN_MIN_RUNNING_PCT) ramped = FAN_MIN_RUNNING_PCT;
        if (ramped > 100.0f)          ramped = 100.0f;
        pct = (int)(ramped + 0.5f);
      }
      break;
    }
  }
  applyFanPercent(pct);
}

void applyFanPercent(int pct) {
  pct = constrain(pct, 0, 100);
  // Enforce fan minimum: anything between 1 and FAN_MIN_RUNNING_PCT - 1
  // becomes FAN_MIN_RUNNING_PCT. 0 (off) is always allowed.
  if (pct > 0 && pct < FAN_MIN_RUNNING_PCT) pct = FAN_MIN_RUNNING_PCT;
  currentFanPct = pct;
  uint32_t duty = map(pct, 0, 100, 0, (1 << FAN_PWM_RESOLUTION) - 1);
  ledcWrite(PWM_PIN, duty);
}

// =========================== Tacho =========================================

void IRAM_ATTR tachoISR() {
  tachoPulseCount++;
}

void updateRPM(unsigned long now) {
  if (now - lastTachoMeasureMs < TACHO_MEASURE_MS) return;
  unsigned long elapsedMs = now - lastTachoMeasureMs;
  lastTachoMeasureMs = now;

  noInterrupts();
  uint32_t pulses = tachoPulseCount;
  tachoPulseCount = 0;
  interrupts();

  if (elapsedMs == 0) return;
  float pps = (float)pulses * 1000.0f / (float)elapsedMs;
  uint16_t rpm = (uint16_t)(pps * 60.0f / TACHO_PULSES_PER_REV);

  // Tiny hysteresis so a single-pulse jitter doesn't trigger a redraw.
  if ((uint16_t)abs((int)rpm - (int)currentRPM) >= TACHO_HYSTERESIS_RPM) {
    currentRPM = rpm;
    if (rpmOverlay && screenState == SCREEN_NORMAL) {
      markDirty();
    }
  } else if (currentFanPct == 0 && currentRPM != 0) {
    currentRPM = 0;
    if (rpmOverlay && screenState == SCREEN_NORMAL) {
      markDirty();
    }
  }
}

// =========================== Backlight / screen state ======================

void setBacklight(int pct) {
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  uint8_t pwm = (uint8_t)((uint32_t)pct * 255 / 100);
  ledcWrite(TFT_BACKLITE, pwm);
}

void enterBrightnessMode(unsigned long now) {
  screenState = SCREEN_BRIGHTNESS;
  brightnessEnteredMs = now;
  markDirty();
}

void exitBrightnessMode() {
  screenState = SCREEN_NORMAL;
  markDirty();
}

void wakeScreen() {
  screenState = SCREEN_NORMAL;
  if (brightnessPct == 0) {
    // Backlight was fully off; restore the brightness target captured when
    // we reached 0. A long-press-bottom jump from a visible level leaves the
    // pre-zero value here; stepping down through the ladder to 0 leaves
    // BRIGHTNESS_FALLBACK_PCT (20%) instead.
    brightnessPct = lastNonZeroBrightness;
    markPrefsDirty();
  }
  setBacklight(brightnessPct);
  markDirty();
}

void sleepScreen() {
  screenState = SCREEN_OFF;
  setBacklight(0);
}

// =========================== Preferences ===================================

void markDirty()      { displayDirty = true; }
void markPrefsDirty() { prefsDirty = true; prefsDirtyMs = millis(); }

void maybeFlushPrefs(unsigned long now) {
  if (!prefsDirty) return;
  if (now - prefsDirtyMs < PREFS_DEBOUNCE_MS) return;
  savePrefs();
  prefsDirty = false;
}

void loadPrefs() {
  prefs.begin("vanfan", true);
  int m = prefs.getInt("mode", MODE_OFF);
  if (m < 0 || m > 2) m = MODE_OFF;
  currentMode = (Mode)m;

  targetTempC = prefs.getFloat("target", DEFAULT_TARGET_C);
  if (targetTempC < MIN_TARGET_C) targetTempC = MIN_TARGET_C;
  if (targetTempC > MAX_TARGET_C) targetTempC = MAX_TARGET_C;

  manualPercent = prefs.getInt("manualPct", DEFAULT_MANUAL_PCT);
  manualPercent = constrain(manualPercent, 0, 100);

  int b = prefs.getInt("brightPct", BRIGHTNESS_DEFAULT_PCT);
  if (b < 0)   b = 0;
  if (b > 100) b = 100;
  brightnessPct = b;

  rpmOverlay = prefs.getBool("rpmOverlay", false);

  prefs.end();
}

void savePrefs() {
  prefs.begin("vanfan", false);
  prefs.putInt("mode",       (int)currentMode);
  prefs.putFloat("target",   targetTempC);
  prefs.putInt("manualPct",  manualPercent);
  prefs.putInt("brightPct", brightnessPct);
  prefs.putBool("rpmOverlay", rpmOverlay);
  prefs.end();
}

// =========================== Display helpers ===============================
// All drawing goes through the canvas, never tft directly. updateDisplay()
// pushes the finished canvas to the TFT in one drawRGBBitmap call.

static void drawCustomCentred(const char* text, int centreX, int baselineY,
                              uint16_t colour, const GFXfont* font) {
  canvas.setFont(font);
  canvas.setTextColor(colour);
  int16_t x1, y1;
  uint16_t w, h;
  canvas.getTextBounds(text, 0, baselineY, &x1, &y1, &w, &h);
  canvas.setCursor(centreX - (w / 2) - x1, baselineY);
  canvas.print(text);
}

static void drawDefaultCentred(const char* text, int centreX, int topY,
                               uint16_t colour, uint8_t size) {
  canvas.setFont(NULL);
  canvas.setTextSize(size);
  canvas.setTextColor(colour);
  int w = (int)strlen(text) * 6 * size;
  canvas.setCursor(centreX - (w / 2), topY);
  canvas.print(text);
}

static void drawDegree(int x, int y, int radius, uint16_t colour) {
  canvas.drawCircle(x, y, radius, colour);
  canvas.drawCircle(x, y, radius - 1, colour);   // thicken
}

static void drawNumberWithDegree(const char* numText, int centreX,
                                 int baselineY, uint16_t colour) {
  const int degreeRadius = 4;
  const int degreeGap    = 5;
  canvas.setFont(&FreeSansBold24pt7b);
  int16_t x1, y1;
  uint16_t w, h;
  canvas.getTextBounds(numText, 0, baselineY, &x1, &y1, &w, &h);
  int totalW = w + degreeGap + degreeRadius * 2;
  int leftX  = centreX - (totalW / 2) - x1;
  canvas.setTextColor(colour);
  canvas.setCursor(leftX, baselineY);
  canvas.print(numText);
  int degreeX = leftX + x1 + w + degreeGap + degreeRadius;
  int degreeY = baselineY - 24;
  drawDegree(degreeX, degreeY, degreeRadius, colour);
}

// =========================== Display update ================================

void updateDisplay() {
  canvas.fillScreen(COLOUR_BG);

  if (screenState == SCREEN_BRIGHTNESS) {
    drawBrightnessScreen();
  } else {
    drawNormalScreen();
  }

  // Single blit to the TFT. Adafruit GFX handles the byte-order conversion.
  tft.drawRGBBitmap(0, 0, canvas.getBuffer(), SCREEN_W, SCREEN_H);
}

// ----- Brightness adjust screen --------------------------------------------

static void drawBrightnessScreen() {
  drawCustomCentred("BRIGHTNESS", SCREEN_W / 2, 32, COLOUR_HEADER, &FreeSansBold12pt7b);

  const int bx = 24, by = 56, bw = SCREEN_W - 2 * bx, bh = 22;
  canvas.drawRect(bx, by, bw, bh, COLOUR_BAR_BRDR);
  canvas.fillRect(bx + 1, by + 1, bw - 2, bh - 2, COLOUR_BAR_BG);
  int fillW = ((bw - 2) * brightnessPct) / 100;
  if (fillW > 0) canvas.fillRect(bx + 1, by + 1, fillW, bh - 2, COLOUR_HEADER);

  char buf[8];
  snprintf(buf, sizeof(buf), "%d%%", brightnessPct);
  drawCustomCentred(buf, SCREEN_W / 2, 112, COLOUR_HEADER, &FreeSansBold12pt7b);
}

// ----- Normal operating screen ---------------------------------------------

static void drawNormalScreen() {
  // ---- Header: mode label ----
  uint16_t modeColour = (currentMode == MODE_OFF) ? COLOUR_LABEL : COLOUR_HEADER;
  drawCustomCentred(MODE_LABELS[currentMode], SCREEN_W / 2,
                    HEADER_BASELINE_Y, modeColour, &FreeSansBold9pt7b);

  // ---- Column divider ----
  canvas.drawFastVLine(DIVIDER_X, DIVIDER_Y_TOP,
                       DIVIDER_Y_BOTTOM - DIVIDER_Y_TOP, COLOUR_DIVIDER);

  // ---- Column labels ----
  drawDefaultCentred("NOW", COL_LEFT_CENTRE_X, LABEL_TOP_Y, COLOUR_LABEL, 1);
  const char* rightLabel = (currentMode == MODE_AUTO) ? "TARGET" : "FAN";
  drawDefaultCentred(rightLabel, COL_RIGHT_CENTRE_X, LABEL_TOP_Y, COLOUR_LABEL, 1);

  // ---- Left big number: current temperature ----
  if (isnan(currentTempC)) {
    drawCustomCentred("--.-", COL_LEFT_CENTRE_X, BIG_BASELINE_Y,
                      COLOUR_ALERT, &FreeSansBold24pt7b);
  } else {
    char tempBuf[16];
    snprintf(tempBuf, sizeof(tempBuf), "%.1f", currentTempC);
    drawNumberWithDegree(tempBuf, COL_LEFT_CENTRE_X, BIG_BASELINE_Y, COLOUR_TEMP);
  }

  // ---- Right big content ----
  switch (currentMode) {
    case MODE_OFF:
      drawCustomCentred("off", COL_RIGHT_CENTRE_X, BIG_BASELINE_Y,
                        COLOUR_MUT, &FreeSansBold18pt7b);
      break;

    case MODE_MANUAL: {
      char buf[8];
      snprintf(buf, sizeof(buf), "%d%%", manualPercent);
      drawCustomCentred(buf, COL_RIGHT_CENTRE_X, BIG_BASELINE_Y,
                        COLOUR_FAN, &FreeSansBold24pt7b);
      break;
    }

    case MODE_AUTO: {
      char buf[8];
      snprintf(buf, sizeof(buf), "%d", (int)(targetTempC + 0.5f));
      drawNumberWithDegree(buf, COL_RIGHT_CENTRE_X, BIG_BASELINE_Y, COLOUR_TARGET);
      break;
    }
  }

  // ---- Bottom row labels ----
  canvas.setFont(NULL);
  canvas.setTextSize(1);
  canvas.setTextColor(COLOUR_LABEL);

  canvas.setCursor(RH_X, LABEL_BOTTOM_Y);
  canvas.print("HUMIDITY");

  const char* fanSpeedLabel = "FAN SPEED";
  int fanSpeedLabelW = (int)strlen(fanSpeedLabel) * 6;
  canvas.setCursor(RIGHT_EDGE_X - fanSpeedLabelW, LABEL_BOTTOM_Y);
  canvas.print(fanSpeedLabel);

  // ---- RH value (bottom-left) ----
  if (isnan(currentRH)) {
    canvas.setFont(&FreeSansBold12pt7b);
    canvas.setTextColor(COLOUR_ALERT);
    canvas.setCursor(RH_X, BOTTOM_BASELINE_Y);
    canvas.print("--");
  } else {
    char rhBuf[8];
    snprintf(rhBuf, sizeof(rhBuf), "%d%%", (int)(currentRH + 0.5f));
    canvas.setFont(&FreeSansBold12pt7b);
    canvas.setTextColor(COLOUR_RH);
    canvas.setCursor(RH_X, BOTTOM_BASELINE_Y);
    canvas.print(rhBuf);
  }

  // ---- Fan bar (now taller to host the RPM overlay) ----
  canvas.drawRect(BAR_X, BAR_Y, BAR_W, BAR_H, COLOUR_BAR_BRDR);
  canvas.fillRect(BAR_X + 1, BAR_Y + 1, BAR_W - 2, BAR_H - 2, COLOUR_BAR_BG);
  if (currentFanPct > 0) {
    int fillW = ((BAR_W - 2) * currentFanPct) / 100;
    canvas.fillRect(BAR_X + 1, BAR_Y + 1, fillW, BAR_H - 2, COLOUR_FAN);
  }

  // ---- RPM overlay ----
  if (rpmOverlay && currentFanPct > 0) {
    char rpmBuf[16];
    if (currentRPM > 0) {
      snprintf(rpmBuf, sizeof(rpmBuf), "%u RPM", currentRPM);
    } else {
      snprintf(rpmBuf, sizeof(rpmBuf), "-- RPM");
    }
    canvas.setFont(&FreeSansBold9pt7b);
    canvas.setTextColor(COLOUR_RPM_TEXT);
    int16_t rx1, ry1;
    uint16_t rw, rh;
    canvas.getTextBounds(rpmBuf, 0, RPM_BASELINE_Y, &rx1, &ry1, &rw, &rh);
    int rpmX = BAR_X + (BAR_W / 2) - (rw / 2) - rx1;
    canvas.setCursor(rpmX, RPM_BASELINE_Y);
    canvas.print(rpmBuf);
  }
}
