// M5DialFanController.ino
// ---------------------------------------------------------------------------
// Campervan PWM fan controller for the M5Stack Dial 1.1.
// Controls two independent fan groups (inside / outside) via PORT.B GPIO.
//
// Hardware: M5Stack Dial 1.1 (ESP32-S3, 1.28" round touch, rotary encoder)
//           Noctua NV-FH2 fan hub(s) or individual PWM fans
//
// Wiring — PORT.B (HY2.0-4P connector):
//   G1  -> Inside fan hub PWM input  (NV-FH2 pin 4, blue wire)
//   G2  -> Outside fan hub PWM input (NV-FH2 pin 4, blue wire)
//   GND -> Fan hub GND               (pin 1, black wire)
//   5V  -> Not connected to fans     (hubs powered via SATA)
//
// Temperature monitoring (future):
//   PORT.A provides I2C (G13 SCL / G15 SDA) for an SHT41 sensor.
//   Use a Grove-to-STEMMA-QT adapter cable for the physical connection.
//
// Controls:
//   Button (press dial)  Cycle mode: OFF → INSIDE → OUTSIDE → ALL → OFF
//   Rotary encoder       Adjust fan speed, brightness, or palette
//   Touch screen         Cycle: main → brightness → palette → main
//
// OFF mode: screen goes dark after 10s; any input wakes to show OFF screen;
//   tap or press from OFF screen turns on (cycles to INSIDE).
//
// Display: Two concentric gradient arcs with selectable colour palettes.
//          Arc spans from 7 o'clock (0%) to 5 o'clock (100%).
//
// Persistence: State saved to NVS on change (debounced 1s), restored at boot.
//
// Required library: M5Dial (includes M5Unified + M5GFX)
// Board: M5Stack Dial (via ESP32 by Espressif board package)
// ---------------------------------------------------------------------------

#include <M5Dial.h>
#include <Preferences.h>
#include <Wire.h>

// =========================== Pin definitions ===============================

#define PWM_INSIDE_PIN    1   // G1 — PORT.B — inside fan group
#define PWM_OUTSIDE_PIN   2   // G2 — PORT.B — outside fan group
#define POWER_HOLD_PIN   46   // Hold HIGH to keep M5Dial powered on
#define PORTA_SDA_PIN    13   // G13 — PORT.A — I2C SDA
#define PORTA_SCL_PIN    15   // G15 — PORT.A — I2C SCL

// =========================== PWM configuration =============================

static const uint32_t FAN_PWM_FREQ = 25000;  // 25 kHz Intel/Noctua spec
static const uint8_t  FAN_PWM_RES  = 8;      // 8-bit duty (0–255)

// =========================== Behaviour tunables ============================

static const int FAN_MIN_PCT      = 20;   // minimum running %; 0 = off
static const int ENCODER_STEP     = 5;    // % change per encoder detent
static const int DEFAULT_INSIDE   = 30;
static const int DEFAULT_OUTSIDE  = 30;
static const int DEFAULT_BRIGHT   = 80;
static const int WAKE_MIN_BRIGHT  = 25;   // floor when turning on

// M5Dial encoder: 64 pulses / 16 physical detents = 4 counts per click.
static const int COUNTS_PER_DETENT = 4;

// =========================== Brightness step ladder ========================

static const int BRIGHT_STEPS[] = { 1, 2, 5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100 };
static const int BRIGHT_STEP_N  = sizeof(BRIGHT_STEPS) / sizeof(BRIGHT_STEPS[0]);

// =========================== Timing ========================================

static const unsigned long DISPLAY_MS         = 33;    // ~30 fps render cap
static const unsigned long BRIGHT_TIMEOUT     = 5000;  // brightness/palette → main
static const unsigned long OFF_SCREEN_TIMEOUT = 10000; // OFF screen → dark
static const unsigned long DIM_TIMEOUT        = 30000; // idle → dim to 2%
static const int           DIM_BRIGHT_PCT     = 2;
static const unsigned long SENSOR_INTERVAL    = 2000;  // read SHT41 every 2s
static const unsigned long PREFS_DEBOUNCE     = 1000;
static const unsigned long TOUCH_DEBOUNCE     = 300;
static const unsigned long ENC_BOUNCE_MS      = 50;    // anti-bounce window

// =========================== Display geometry ==============================

static const int16_t CX = 120, CY = 120;          // screen centre

static const int16_t OUT_R1 = 116, OUT_R2 = 100;  // outer ring radii
static const int16_t IN_R1  = 94,  IN_R2  = 78;   // inner ring radii

// Brightness arc: half-width, centred in the same radial space
static const int16_t BR_R1 = 107, BR_R2 = 88;

// LovyanGFX angles: 0° = 3 o'clock (east), positive = clockwise.
// 7 o'clock = 120°, sweep 300° clockwise to 5 o'clock (60°).
static const float ARC_START = 120.0f;
static const float ARC_SWEEP = 300.0f;

// =========================== Colours =======================================

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
}

static const uint16_t COL_BG        = rgb565(0,   0,   0);
static const uint16_t COL_TRACK     = rgb565(20,  20,  20);
static const uint16_t COL_TRACK_OFF = rgb565(10,  10,  10);
static const uint16_t COL_TEXT      = rgb565(230, 230, 230);
static const uint16_t COL_LABEL     = rgb565(110, 110, 110);
static const uint16_t COL_OFF       = rgb565(65,  65,  65);
static const uint16_t COL_TICK      = rgb565(55,  55,  55);

// =========================== Palettes ======================================

struct Palette { float inH0, inH1, outH0, outH1; bool solid; };

static const int PAL_COUNT = 18;
static const Palette PALETTES[PAL_COUNT] = {
  // Hue-shift gradients (brightness ramps 0.7→1.0 along arc)
  { 230, 185, 120, 60,  false },  // Blue→Cyan / Green→Yellow
  { 320, 280,  30,  0,  false },  // Pink→Purple / Orange→Red
  { 200, 160,  15, 40,  false },  // Teal→Mint / Scarlet→Amber
  { 260, 220,  50, 25,  false },  // Violet→Blue / Gold→Coral
  { 170, 130, 340, 310, false },  // Cyan→Green / Rose→Magenta
  { 300, 260,  55, 30,  false },  // Magenta→Indigo / Yellow→Orange
  { 210, 250,  45, 60,  false },  // Blue→Indigo / Amber→Yellow
  { 150, 100, 330, 295, false },  // Teal→Lime / Pink→Violet
  { 185, 225, 310, 340, false },  // Cyan→Blue / Purple→Rose
  { 240, 175,  10, 50,  false },  // Indigo→Teal / Red→Gold

  // Solid colours (uniform brightness, two shades per colour family)
  { 350, 350,  10,  10, true  },  // Red (crimson / scarlet)
  { 210, 210, 245, 245, true  },  // Blue (sky / indigo)
  {  25,  25,  42,  42, true  },  // Orange (orange / amber)
  { 270, 270, 305, 305, true  },  // Purple (violet / magenta)

  // Brightness-ramp (single hue, brightness ramps along arc)
  { 350, 350,  10,  10, false },  // Red ramp
  { 210, 210, 245, 245, false },  // Blue ramp
  {  25,  25,  42,  42, false },  // Orange ramp
  { 270, 270, 305, 305, false },  // Purple ramp
};

// =========================== State =========================================

enum Mode   : uint8_t { MODE_OFF = 0, MODE_INSIDE = 1, MODE_OUTSIDE = 2, MODE_BOTH = 3 };
enum Screen : uint8_t { SCR_MAIN = 0, SCR_BRIGHT = 1, SCR_PALETTE = 2, SCR_OFF = 3 };

static const char* MODE_NAMES[] = { "OFF", "INSIDE", "OUTSIDE", "ALL" };

Mode      mode      = MODE_OFF;
int       inPct     = DEFAULT_INSIDE;
int       outPct    = DEFAULT_OUTSIDE;
Screen    scr       = SCR_OFF;
int       brightPct = DEFAULT_BRIGHT;
int       palIdx    = 0;

long          encLast        = 0;
long          encAccum       = 0;
int           lastEncDir     = 0;
unsigned long lastEncEventMs = 0;
bool          dirty          = true;
bool          prefsDirty     = false;
unsigned long prefsDirtyMs   = 0;
unsigned long lastDrawMs     = 0;
unsigned long brightEnterMs  = 0;
unsigned long offScreenMs    = 0;
unsigned long lastTouchMs    = 0;
bool          wasTouching    = false;
unsigned long lastActivityMs = 0;
bool          dimmed         = false;

static const uint8_t SHT41_ADDR = 0x44;
bool          sensorOk       = false;
float         tempC          = 0;
float         humidity       = 0;
unsigned long lastSensorMs   = 0;

LGFX_Sprite canvas(&M5Dial.Display);
Preferences prefs;

// =========================== Forward declarations ==========================

static uint16_t hsvTo565(float h, float s, float v);
static void drawGradientArc(int16_t r1, int16_t r2, int pct,
                            bool active, float h0, float h1, bool solid);
static void drawTick(float angleDeg, int16_t rIn, int16_t rOut, uint16_t col);
static void drawMainScreen();
static void drawBrightnessScreen();
static void drawPaletteScreen();
static int  clampFan(int pct);
static void applyFanPWM();
static void handleEncoder();
static void handleTouch();
static void handleButton();
static void enterBrightness();
static void sleepScreen();
static void turnOn();
static void markDirty();
static void markPrefsDirty();
static void loadPrefs();
static void savePrefs();
static int  nextBrightStep(int cur);
static int  prevBrightStep(int cur);
static void resetActivity();
static void readSensor();

// =========================== Setup =========================================

void setup() {
  Serial.begin(115200);

  pinMode(POWER_HOLD_PIN, OUTPUT);
  digitalWrite(POWER_HOLD_PIN, HIGH);

  auto cfg = M5.config();
  M5Dial.begin(cfg, true, false);
  M5Dial.Speaker.setVolume(255);

  canvas.setColorDepth(16);
  canvas.createSprite(240, 240);

  ledcAttach(PWM_INSIDE_PIN,  FAN_PWM_FREQ, FAN_PWM_RES);
  ledcAttach(PWM_OUTSIDE_PIN, FAN_PWM_FREQ, FAN_PWM_RES);

  // PORT.A I2C: must init with swapped pins first to release GPIOs from
  // M5Dial's internal I2C driver, then re-init with correct assignment.
  Wire1.begin(PORTA_SCL_PIN, PORTA_SDA_PIN);
  Wire1.end();
  Wire1.begin(PORTA_SDA_PIN, PORTA_SCL_PIN);
  Wire1.beginTransmission(SHT41_ADDR);
  sensorOk = (Wire1.endTransmission() == 0);
  if (sensorOk) {
    Wire1.beginTransmission(SHT41_ADDR);
    Wire1.write(0x94);  // soft reset
    Wire1.endTransmission();
    delay(1);
    Serial.println("SHT41 found on PORT.A");
  } else {
    Serial.println("SHT41 not found — running without sensor");
  }

  loadPrefs();

  if (mode == MODE_OFF) {
    scr = SCR_OFF;
    M5Dial.Display.setBrightness(0);
  } else {
    scr = SCR_MAIN;
    brightPct = max(brightPct, WAKE_MIN_BRIGHT);
    M5Dial.Display.setBrightness(brightPct * 255 / 100);
  }

  encLast = M5Dial.Encoder.read();
  lastActivityMs = millis();
  applyFanPWM();
  Serial.println("M5Dial Fan Controller ready");
}

// =========================== Main loop =====================================

void loop() {
  unsigned long now = millis();
  M5Dial.update();

  handleButton();
  handleEncoder();
  handleTouch();

  now = millis();

  // Read temperature sensor periodically
  if (sensorOk && (now - lastSensorMs) >= SENSOR_INTERVAL) {
    lastSensorMs = now;
    readSensor();
  }

  // Brightness / palette timeout → return to main
  if ((scr == SCR_BRIGHT || scr == SCR_PALETTE)
      && (now - brightEnterMs) >= BRIGHT_TIMEOUT) {
    scr = SCR_MAIN;
    lastActivityMs = now;
    markDirty();
  }

  // OFF-mode idle → darken screen
  if (scr == SCR_MAIN && mode == MODE_OFF
      && (now - offScreenMs) >= OFF_SCREEN_TIMEOUT) {
    sleepScreen();
  }

  // Auto-dim after idle (not while adjusting brightness or browsing palettes)
  if (!dimmed && scr == SCR_MAIN
      && (now - lastActivityMs) >= DIM_TIMEOUT) {
    dimmed = true;
    M5Dial.Display.setBrightness(DIM_BRIGHT_PCT * 255 / 100);
  }

  // Flush prefs after debounce
  if (prefsDirty && (now - prefsDirtyMs) >= PREFS_DEBOUNCE) {
    savePrefs();
    prefsDirty = false;
  }

  // Render
  if (scr != SCR_OFF && dirty && (now - lastDrawMs) >= DISPLAY_MS) {
    lastDrawMs = now;
    dirty = false;
    canvas.fillSprite(COL_BG);
    switch (scr) {
      case SCR_BRIGHT:  drawBrightnessScreen(); break;
      case SCR_PALETTE: drawPaletteScreen();    break;
      default:          drawMainScreen();       break;
    }
    canvas.pushSprite(0, 0);
  }
}

// =========================== Fan PWM =======================================

static int clampFan(int pct) {
  if (pct <= 0) return 0;
  if (pct < FAN_MIN_PCT) return FAN_MIN_PCT;
  if (pct > 100) return 100;
  return pct;
}

static void applyFanPWM() {
  uint32_t inD = 0, outD = 0;
  if (mode == MODE_INSIDE || mode == MODE_BOTH)
    inD = map(clampFan(inPct), 0, 100, 0, 255);
  if (mode == MODE_OUTSIDE || mode == MODE_BOTH)
    outD = map(clampFan(outPct), 0, 100, 0, 255);
  ledcWrite(PWM_INSIDE_PIN,  inD);
  ledcWrite(PWM_OUTSIDE_PIN, outD);
}

// =========================== Input helpers =================================

static void markDirty()      { dirty = true; }
static void markPrefsDirty() { prefsDirty = true; prefsDirtyMs = millis(); }

static void resetActivity() {
  lastActivityMs = millis();
  if (dimmed) {
    dimmed = false;
    M5Dial.Display.setBrightness(brightPct * 255 / 100);
  }
}

static void readSensor() {
  Wire1.beginTransmission(SHT41_ADDR);
  Wire1.write(0xFD);  // high-precision, no heater
  Wire1.endTransmission();
  delay(10);
  if (Wire1.requestFrom(SHT41_ADDR, (uint8_t)6) == 6) {
    uint8_t buf[6];
    for (int i = 0; i < 6; i++) buf[i] = Wire1.read();
    float tRaw = (uint16_t)buf[0] << 8 | buf[1];
    float hRaw = (uint16_t)buf[3] << 8 | buf[4];
    tempC    = -45.0f + 175.0f * tRaw / 65535.0f;
    humidity = constrain(-6.0f + 125.0f * hRaw / 65535.0f, 0.0f, 100.0f);
    if (scr == SCR_MAIN) markDirty();
  }
}

static int nextBrightStep(int cur) {
  for (int i = 0; i < BRIGHT_STEP_N; i++)
    if (BRIGHT_STEPS[i] > cur) return BRIGHT_STEPS[i];
  return BRIGHT_STEPS[BRIGHT_STEP_N - 1];
}

static int prevBrightStep(int cur) {
  for (int i = BRIGHT_STEP_N - 1; i >= 0; i--)
    if (BRIGHT_STEPS[i] < cur) return BRIGHT_STEPS[i];
  return BRIGHT_STEPS[0];
}

// =========================== Input handlers ================================

static void handleEncoder() {
  long pos = M5Dial.Encoder.read();
  long rawDelta = pos - encLast;
  if (rawDelta == 0) return;

  encAccum += rawDelta;
  encLast = pos;

  int detents = encAccum / COUNTS_PER_DETENT;
  if (detents == 0) return;
  encAccum -= detents * COUNTS_PER_DETENT;

  // Anti-bounce: suppress rapid direction reversals
  unsigned long now = millis();
  int dir = (detents > 0) ? 1 : -1;
  if (dir != lastEncDir && (now - lastEncEventMs) < ENC_BOUNCE_MS) return;
  lastEncDir = dir;
  lastEncEventMs = now;

  resetActivity();

  if (scr == SCR_OFF || mode == MODE_OFF) { turnOn(); return; }

  if (scr == SCR_BRIGHT) {
    for (int i = 0; i < abs(detents); i++)
      brightPct = (detents > 0) ? nextBrightStep(brightPct) : prevBrightStep(brightPct);
    M5Dial.Display.setBrightness(brightPct * 255 / 100);
    brightEnterMs = millis();
    markDirty();
    markPrefsDirty();
    return;
  }

  if (scr == SCR_PALETTE) {
    palIdx = ((palIdx + detents) % PAL_COUNT + PAL_COUNT) % PAL_COUNT;
    brightEnterMs = millis();
    markDirty();
    markPrefsDirty();
    return;
  }

  int change = detents * ENCODER_STEP;
  if (mode == MODE_INSIDE || mode == MODE_BOTH)
    inPct = constrain(inPct + change, 0, 100);
  if (mode == MODE_OUTSIDE || mode == MODE_BOTH)
    outPct = constrain(outPct + change, 0, 100);

  applyFanPWM();
  markDirty();
  markPrefsDirty();
}

static void handleTouch() {
  bool touching = M5Dial.Touch.getCount() > 0;

  if (touching && !wasTouching) {
    unsigned long now = millis();
    if (now - lastTouchMs >= TOUCH_DEBOUNCE) {
      lastTouchMs = now;
      resetActivity();

      if (scr == SCR_OFF || mode == MODE_OFF) {
        turnOn();
      } else if (scr == SCR_MAIN) {
        enterBrightness();
        M5Dial.Speaker.tone(2000, 25);
      } else if (scr == SCR_BRIGHT) {
        scr = SCR_PALETTE;
        brightEnterMs = millis();
        markDirty();
        M5Dial.Speaker.tone(2500, 25);
      } else if (scr == SCR_PALETTE) {
        scr = SCR_MAIN;
        markDirty();
        M5Dial.Speaker.tone(1000, 25);
      }
    }
  }
  wasTouching = touching;
}

static void handleButton() {
  // Long press → turn off; flag prevents the subsequent release from turning back on
  static bool holdFired = false;
  if (M5Dial.BtnA.wasHold()) {
    holdFired = true;
    resetActivity();
    mode = MODE_OFF;
    applyFanPWM();
    offScreenMs = millis();
    scr = SCR_MAIN;
    markDirty();
    markPrefsDirty();
    M5Dial.Speaker.tone(800, 60);
    return;
  }

  if (!M5Dial.BtnA.wasReleased()) return;
  if (holdFired) { holdFired = false; return; }
  resetActivity();

  if (scr == SCR_OFF) { turnOn(); return; }

  if (mode == MODE_OFF) { turnOn(); return; }

  if (scr == SCR_BRIGHT || scr == SCR_PALETTE) {
    scr = SCR_MAIN;
    markDirty();
    M5Dial.Speaker.tone(1000, 25);
    return;
  }

  // Cycle mode: INSIDE → OUTSIDE → ALL → INSIDE
  if      (mode == MODE_INSIDE)  mode = MODE_OUTSIDE;
  else if (mode == MODE_OUTSIDE) mode = MODE_BOTH;
  else                           mode = MODE_INSIDE;

  applyFanPWM();
  markDirty();
  markPrefsDirty();
  M5Dial.Speaker.tone(1500, 40);
}

// =========================== Screen state ==================================

static void enterBrightness() {
  scr = SCR_BRIGHT;
  brightEnterMs = millis();
  markDirty();
}

static void turnOn() {
  mode = MODE_INSIDE;
  scr = SCR_MAIN;
  brightPct = max(brightPct, WAKE_MIN_BRIGHT);
  applyFanPWM();
  canvas.fillSprite(COL_BG);
  drawMainScreen();
  canvas.pushSprite(0, 0);
  M5Dial.Display.setBrightness(brightPct * 255 / 100);
  dirty = false;
  markPrefsDirty();
  M5Dial.Speaker.tone(1500, 40);
}

static void sleepScreen() {
  scr = SCR_OFF;
  canvas.fillSprite(0);
  canvas.pushSprite(0, 0);
  M5Dial.Display.setBrightness(0);
  dirty = false;
}

// =========================== Preferences ===================================

static void loadPrefs() {
  prefs.begin("dialfan", true);
  mode      = (Mode)constrain(prefs.getInt("mode",   MODE_OFF), 0, 3);
  inPct     = constrain(prefs.getInt("inPct",  DEFAULT_INSIDE),  0, 100);
  outPct    = constrain(prefs.getInt("outPct", DEFAULT_OUTSIDE), 0, 100);
  brightPct = constrain(prefs.getInt("bright", DEFAULT_BRIGHT),  1, 100);
  palIdx    = constrain(prefs.getInt("palette", 0), 0, PAL_COUNT - 1);
  prefs.end();
}

static void savePrefs() {
  prefs.begin("dialfan", false);
  prefs.putInt("mode",    (int)mode);
  prefs.putInt("inPct",   inPct);
  prefs.putInt("outPct",  outPct);
  prefs.putInt("bright",  brightPct);
  prefs.putInt("palette", palIdx);
  prefs.end();
}

// =========================== Colour helpers ================================

static uint16_t hsvTo565(float h, float s, float v) {
  while (h < 0)    h += 360;
  while (h >= 360) h -= 360;
  float c = v * s;
  float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
  float m = v - c;
  float r, g, b;
  if      (h <  60) { r = c; g = x; b = 0; }
  else if (h < 120) { r = x; g = c; b = 0; }
  else if (h < 180) { r = 0; g = c; b = x; }
  else if (h < 240) { r = 0; g = x; b = c; }
  else if (h < 300) { r = x; g = 0; b = c; }
  else              { r = c; g = 0; b = x; }
  return rgb565((uint8_t)((r + m) * 255),
                (uint8_t)((g + m) * 255),
                (uint8_t)((b + m) * 255));
}

// =========================== Drawing helpers ===============================

// Tick marks use the same coordinate system as fillArc:
// 0° = 3 o'clock (east), clockwise positive.
static void drawTick(float angleDeg, int16_t rIn, int16_t rOut, uint16_t col) {
  float rad = angleDeg * PI / 180.0f;
  float cs = cosf(rad), sn = sinf(rad);
  canvas.drawLine(CX + (int)(rIn  * cs), CY + (int)(rIn  * sn),
                  CX + (int)(rOut * cs), CY + (int)(rOut * sn), col);
}

static void drawGradientArc(int16_t r1, int16_t r2, int pct,
                            bool active, float h0, float h1, bool solid) {
  uint16_t trackCol = active ? COL_TRACK : COL_TRACK_OFF;
  canvas.fillArc(CX, CY, r1, r2, ARC_START, ARC_START + ARC_SWEEP, trackCol);

  if (active && pct > 0) {
    float sweep = ARC_SWEEP * pct / 100.0f;
    const float seg = 3.0f;
    int count = (int)ceilf(sweep / seg);
    for (int i = 0; i < count; i++) {
      float a0 = ARC_START + i * seg;
      float a1 = ARC_START + fminf((i + 1) * seg, sweep);
      float t = (a0 - ARC_START) / ARC_SWEEP;
      float a1_draw = (i < count - 1) ? a1 + 1.5f : a1;
      float v = solid ? 1.0f : (0.7f + 0.3f * t);
      canvas.fillArc(CX, CY, r1, r2, a0, a1_draw,
                     hsvTo565(h0 + (h1 - h0) * t, 1.0f, v));
    }
  }
}

// =========================== Screen drawing ================================

static void drawMainScreen() {
  const Palette& pal = PALETTES[palIdx];
  bool active = (mode != MODE_OFF);

  drawGradientArc(OUT_R1, OUT_R2, active ? outPct : 0, active,
                  pal.outH0, pal.outH1, pal.solid);
  drawGradientArc(IN_R1,  IN_R2,  active ? inPct  : 0, active,
                  pal.inH0, pal.inH1, pal.solid);

  for (int i = 0; i <= 4; i++) {
    float angle = ARC_START + ARC_SWEEP * i / 4.0f;
    drawTick(angle, OUT_R1 + 1, OUT_R1 + 5, COL_TICK);
  }

  canvas.setTextDatum(middle_center);

  if (sensorOk) {
    float avgH = (pal.inH0 + pal.inH1 + pal.outH0 + pal.outH1) * 0.25f;
    float compH = fmodf(avgH + 180.0f, 360.0f);
    uint16_t tempCol = hsvTo565(compH, 0.5f, 0.8f);
    char tbuf[10];
    snprintf(tbuf, sizeof(tbuf), "%.1f\xC2\xB0""C", tempC);
    canvas.setFont(&fonts::lv_font_montserrat_24);
    canvas.setTextColor(tempCol);
    canvas.drawString(tbuf, CX, CY + 98);
  }

  if (mode == MODE_OFF) {
    canvas.setFont(&fonts::lv_font_montserrat_28);
    canvas.setTextColor(COL_OFF);
    canvas.drawString("OFF", CX, CY - 5);

    canvas.setFont(&fonts::lv_font_montserrat_12);
    canvas.setTextColor(COL_LABEL);
    canvas.drawString("tap to start", CX, CY + 28);
    return;
  }

  float midV = pal.solid ? 1.0f : 0.85f;
  uint16_t accentCol;
  if (mode == MODE_INSIDE)
    accentCol = hsvTo565((pal.inH0 + pal.inH1) * 0.5f, 1.0f, midV);
  else if (mode == MODE_OUTSIDE)
    accentCol = hsvTo565((pal.outH0 + pal.outH1) * 0.5f, 1.0f, midV);
  else
    accentCol = 0xFFFF;

  canvas.setFont(&fonts::lv_font_montserrat_16);
  canvas.setTextColor(accentCol);
  canvas.drawString(MODE_NAMES[mode], CX, CY - 30);

  int dispPct;
  if      (mode == MODE_INSIDE)  dispPct = inPct;
  else if (mode == MODE_OUTSIDE) dispPct = outPct;
  else                           dispPct = (inPct + outPct + 1) / 2;

  char buf[8];
  snprintf(buf, sizeof(buf), "%d%%", dispPct);
  canvas.setFont(&fonts::lv_font_montserrat_36);
  canvas.setTextColor(accentCol);
  canvas.drawString(buf, CX, CY + 8);

  if (mode == MODE_BOTH || inPct != outPct) {
    canvas.setFont(&fonts::lv_font_montserrat_12);

    char inBuf[12], outBuf[12];
    snprintf(inBuf,  sizeof(inBuf),  "IN %d%%",  inPct);
    snprintf(outBuf, sizeof(outBuf), "OUT %d%%", outPct);

    int inW  = canvas.textWidth(inBuf);
    int outW = canvas.textWidth(outBuf);
    int gap  = 8;
    int startX = CX - (inW + gap + outW) / 2;

    canvas.setTextDatum(middle_left);

    canvas.setTextColor(hsvTo565((pal.inH0 + pal.inH1) * 0.5f, 1.0f, midV));
    canvas.drawString(inBuf, startX, CY + 40);

    canvas.setTextColor(hsvTo565((pal.outH0 + pal.outH1) * 0.5f, 1.0f, midV));
    canvas.drawString(outBuf, startX + inW + gap, CY + 40);

    canvas.setTextDatum(middle_center);
  }

}

static void drawBrightnessScreen() {
  canvas.fillArc(CX, CY, BR_R1, BR_R2, ARC_START,
                 ARC_START + ARC_SWEEP, COL_TRACK);

  if (brightPct > 0) {
    float sweep = ARC_SWEEP * brightPct / 100.0f;
    const float seg = 3.0f;
    int count = (int)ceilf(sweep / seg);
    for (int i = 0; i < count; i++) {
      float a0 = ARC_START + i * seg;
      float a1 = ARC_START + fminf((i + 1) * seg, sweep);
      float t  = (a0 - ARC_START) / ARC_SWEEP;
      float a1_draw = (i < count - 1) ? a1 + 1.5f : a1;
      uint8_t gray = (uint8_t)(40 + 215 * t);
      canvas.fillArc(CX, CY, BR_R1, BR_R2, a0, a1_draw,
                     rgb565(gray, gray, gray));
    }
  }

  for (int i = 0; i <= 4; i++) {
    float angle = ARC_START + ARC_SWEEP * i / 4.0f;
    drawTick(angle, OUT_R1 + 1, OUT_R1 + 5, COL_TICK);
  }

  canvas.setTextDatum(middle_center);

  canvas.setFont(&fonts::lv_font_montserrat_16);
  canvas.setTextColor(COL_TEXT);
  canvas.drawString("BRIGHTNESS", CX, CY - 25);

  char buf[8];
  snprintf(buf, sizeof(buf), "%d%%", brightPct);
  canvas.setFont(&fonts::lv_font_montserrat_36);
  canvas.setTextColor(COL_TEXT);
  canvas.drawString(buf, CX, CY + 12);
}

static void drawPaletteScreen() {
  const Palette& pal = PALETTES[palIdx];

  drawGradientArc(OUT_R1, OUT_R2, 100, true, pal.outH0, pal.outH1, pal.solid);
  drawGradientArc(IN_R1,  IN_R2,  100, true, pal.inH0,  pal.inH1,  pal.solid);

  for (int i = 0; i <= 4; i++) {
    float angle = ARC_START + ARC_SWEEP * i / 4.0f;
    drawTick(angle, OUT_R1 + 1, OUT_R1 + 5, COL_TICK);
  }

  canvas.setTextDatum(middle_center);

  canvas.setFont(&fonts::lv_font_montserrat_16);
  canvas.setTextColor(COL_LABEL);
  canvas.drawString("PALETTE", CX, CY - 25);

  char buf[8];
  snprintf(buf, sizeof(buf), "%d/%d", palIdx + 1, PAL_COUNT);
  canvas.setFont(&fonts::lv_font_montserrat_24);
  canvas.setTextColor(COL_TEXT);
  canvas.drawString(buf, CX, CY + 12);
}
