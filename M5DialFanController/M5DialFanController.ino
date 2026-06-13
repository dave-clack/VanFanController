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
//   Rotary encoder     Adjust fan speed or brightness
//   Touch outer ring   Select outside fans
//   Touch inner ring   Select inside fans
//   Touch centre       Select all fans (linked)
//   Button short       Toggle fans on/off
//   Button long        Enter brightness mode
//
// Brightness mode:
//   Encoder            Adjust brightness
//   Button / touch     Exit to main
//   5s timeout         Screen off; any input wakes
//
// Display: Two concentric gradient arcs (blue → red) with indicator dots
//          and selection highlights on a dark theme.
//
// Persistence: State saved to NVS on change (debounced 1s), restored at boot.
//
// Required library: M5Dial (includes M5Unified + M5GFX)
// Board: M5Stack Dial (via ESP32 by Espressif board package)
// ---------------------------------------------------------------------------

#include <M5Dial.h>
#include <Preferences.h>

// =========================== Pin definitions ===============================

#define PWM_INSIDE_PIN    1   // G1 — PORT.B — inside fan group
#define PWM_OUTSIDE_PIN   2   // G2 — PORT.B — outside fan group
#define POWER_HOLD_PIN   46   // Hold HIGH to keep M5Dial powered on

// =========================== PWM configuration =============================

static const uint32_t FAN_PWM_FREQ = 25000;  // 25 kHz Intel/Noctua spec
static const uint8_t  FAN_PWM_RES  = 8;      // 8-bit duty (0–255)

// =========================== Behaviour tunables ============================

// Minimum non-zero fan speed. Any requested speed between 1% and this value
// is clamped up to this value; 0% is always allowed and means "stopped".
// 20% is safe for most Noctua 12V fans; lower if your fans start reliably
// at a lower duty.
static const int FAN_MIN_PCT      = 20;

static const int ENCODER_STEP     = 5;    // % change per encoder detent
static const int BRIGHTNESS_MIN   = 5;    // lowest screen brightness %
static const int BRIGHTNESS_MAX   = 100;
static const int DEFAULT_INSIDE   = 30;
static const int DEFAULT_OUTSIDE  = 30;
static const int DEFAULT_BRIGHT   = 80;

// Encoder counts per physical detent. The M5Dial typically reports 1 count
// per detent, but if turning feels too sensitive or sluggish, try 2 or 4.
static const int COUNTS_PER_DETENT = 1;

// =========================== Timing ========================================

static const unsigned long DISPLAY_MS     = 33;    // ~30 fps render cap
static const unsigned long LONG_PRESS_MS  = 800;
static const unsigned long BRIGHT_TIMEOUT = 5000;  // brightness mode → off
static const unsigned long PREFS_DEBOUNCE = 1000;
static const unsigned long TOUCH_DEBOUNCE = 250;

// =========================== Display geometry ==============================

static const int16_t CX = 120, CY = 120;          // screen centre

static const int16_t OUT_R1 = 116, OUT_R2 = 100;  // outer ring radii
static const int16_t IN_R1  = 94,  IN_R2  = 78;   // inner ring radii

// 270° gauge arc: 7:30 position clockwise to 4:30 position.
// LovyanGFX angles: 0° = 12 o'clock, positive = clockwise.
static const float ARC_START = 225.0f;
static const float ARC_SWEEP = 270.0f;

// Touch hit-test: distance from centre
static const float TOUCH_OUTER_MIN = 80.0f;  // ≥ → outside ring
static const float TOUCH_INNER_MIN = 40.0f;  // ≥ → inside ring
                                               // < → centre (all)

// =========================== Colours =======================================

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
}

static const uint16_t COL_BG        = rgb565(6,   6,   16);
static const uint16_t COL_TRACK     = rgb565(22,  22,  38);
static const uint16_t COL_TRACK_SEL = rgb565(35,  35,  55);
static const uint16_t COL_TRACK_OFF = rgb565(12,  12,  22);
static const uint16_t COL_TEXT      = rgb565(230, 230, 240);
static const uint16_t COL_LABEL     = rgb565(100, 100, 120);
static const uint16_t COL_ACCENT    = rgb565(0,   188, 212);
static const uint16_t COL_OFF       = rgb565(60,  60,  70);
static const uint16_t COL_SEL_RING  = rgb565(180, 180, 200);
static const uint16_t COL_AMBER     = rgb565(255, 193, 7);
static const uint16_t COL_TICK      = rgb565(50,  50,  65);
static const uint16_t COL_DOT_EDGE  = rgb565(255, 255, 255);

// =========================== State =========================================

enum FanGroup : uint8_t { GRP_INSIDE = 0, GRP_OUTSIDE = 1, GRP_ALL = 2 };
enum Screen   : uint8_t { SCR_MAIN = 0, SCR_BRIGHT = 1, SCR_OFF = 2 };

static const char* GRP_NAMES[] = { "INSIDE", "OUTSIDE", "ALL" };

bool      fansOn    = false;
int       inPct     = DEFAULT_INSIDE;
int       outPct    = DEFAULT_OUTSIDE;
FanGroup  selGrp    = GRP_ALL;
Screen    scr       = SCR_MAIN;
int       brightPct = DEFAULT_BRIGHT;

long          encLast       = 0;
long          encAccum      = 0;
bool          dirty         = true;
bool          prefsDirty    = false;
unsigned long prefsDirtyMs  = 0;
unsigned long lastDrawMs    = 0;
unsigned long brightEnterMs = 0;
unsigned long lastTouchMs   = 0;
bool          btnLongFired  = false;
bool          wasTouching   = false;

LGFX_Sprite canvas(&M5Dial.Display);
Preferences prefs;

// =========================== Forward declarations ==========================

static uint16_t hsvTo565(float h, float s, float v);
static uint16_t speedColor(float t);
static void drawGradientArc(int16_t r1, int16_t r2, int pct,
                            bool selected, bool systemOn);
static void drawIndicatorDot(int pct, int16_t r1, int16_t r2);
static void drawTick(float angleDeg, int16_t rIn, int16_t rOut, uint16_t col);
static void drawMainScreen();
static void drawBrightnessScreen();
static int  clampFan(int pct);
static void applyFanPWM();
static void handleEncoder();
static void handleTouch();
static void handleButton();
static void enterBrightness();
static void wakeScreen();
static void sleepScreen();
static void markDirty();
static void markPrefsDirty();
static void loadPrefs();
static void savePrefs();

// =========================== Setup =========================================

void setup() {
  Serial.begin(115200);

  pinMode(POWER_HOLD_PIN, OUTPUT);
  digitalWrite(POWER_HOLD_PIN, HIGH);

  auto cfg = M5.config();
  M5Dial.begin(cfg, true, false);

  canvas.setColorDepth(16);
  canvas.createSprite(240, 240);

  ledcAttach(PWM_INSIDE_PIN,  FAN_PWM_FREQ, FAN_PWM_RES);
  ledcAttach(PWM_OUTSIDE_PIN, FAN_PWM_FREQ, FAN_PWM_RES);

  loadPrefs();
  M5Dial.Display.setBrightness(brightPct * 255 / 100);
  encLast = M5Dial.Encoder.read();
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

  if (scr == SCR_BRIGHT && (now - brightEnterMs) >= BRIGHT_TIMEOUT) {
    sleepScreen();
  }

  if (prefsDirty && (now - prefsDirtyMs) >= PREFS_DEBOUNCE) {
    savePrefs();
    prefsDirty = false;
  }

  if (scr != SCR_OFF && dirty && (now - lastDrawMs) >= DISPLAY_MS) {
    lastDrawMs = now;
    dirty = false;
    canvas.fillSprite(COL_BG);
    if (scr == SCR_BRIGHT) drawBrightnessScreen();
    else                    drawMainScreen();
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
  if (fansOn) {
    inD  = map(clampFan(inPct),  0, 100, 0, 255);
    outD = map(clampFan(outPct), 0, 100, 0, 255);
  }
  ledcWrite(PWM_INSIDE_PIN,  inD);
  ledcWrite(PWM_OUTSIDE_PIN, outD);
}

// =========================== Input handlers ================================

static void markDirty()      { dirty = true; }
static void markPrefsDirty() { prefsDirty = true; prefsDirtyMs = millis(); }

static void handleEncoder() {
  long pos = M5Dial.Encoder.read();
  long rawDelta = pos - encLast;
  if (rawDelta == 0) return;

  encAccum += rawDelta;
  encLast = pos;

  int detents = encAccum / COUNTS_PER_DETENT;
  if (detents == 0) return;
  encAccum -= detents * COUNTS_PER_DETENT;

  if (scr == SCR_OFF) { wakeScreen(); return; }

  if (scr == SCR_BRIGHT) {
    brightPct = constrain(brightPct + detents * 5, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
    M5Dial.Display.setBrightness(brightPct * 255 / 100);
    brightEnterMs = millis();
    markDirty();
    markPrefsDirty();
    return;
  }

  if (!fansOn) return;

  int change = detents * ENCODER_STEP;
  if (selGrp == GRP_INSIDE || selGrp == GRP_ALL)
    inPct = constrain(inPct + change, 0, 100);
  if (selGrp == GRP_OUTSIDE || selGrp == GRP_ALL)
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

      if (scr == SCR_OFF)    { wakeScreen();  wasTouching = touching; return; }
      if (scr == SCR_BRIGHT) { scr = SCR_MAIN; markDirty(); wasTouching = touching; return; }

      auto t = M5Dial.Touch.getDetail();
      float dx = t.x - CX;
      float dy = t.y - CY;
      float dist = sqrtf(dx * dx + dy * dy);

      FanGroup prev = selGrp;
      if      (dist >= TOUCH_OUTER_MIN) selGrp = GRP_OUTSIDE;
      else if (dist >= TOUCH_INNER_MIN) selGrp = GRP_INSIDE;
      else                               selGrp = GRP_ALL;

      if (selGrp != prev) {
        M5Dial.Speaker.tone(2000, 20);
        markDirty();
        markPrefsDirty();
      }
    }
  }
  wasTouching = touching;
}

static void handleButton() {
  if (M5Dial.BtnA.wasPressed()) {
    btnLongFired = false;
  }

  if (M5Dial.BtnA.isPressed() && !btnLongFired
      && M5Dial.BtnA.pressedFor(LONG_PRESS_MS)) {
    btnLongFired = true;
    if (scr == SCR_OFF) { wakeScreen(); return; }
    if (scr == SCR_MAIN) {
      enterBrightness();
      M5Dial.Speaker.tone(1000, 40);
    }
    return;
  }

  if (M5Dial.BtnA.wasReleased() && !btnLongFired) {
    if (scr == SCR_OFF)    { wakeScreen(); return; }
    if (scr == SCR_BRIGHT) { scr = SCR_MAIN; markDirty(); return; }

    fansOn = !fansOn;
    applyFanPWM();
    markDirty();
    markPrefsDirty();
    M5Dial.Speaker.tone(fansOn ? 1500 : 800, 40);
  }
}

// =========================== Screen state ==================================

static void enterBrightness() {
  scr = SCR_BRIGHT;
  brightEnterMs = millis();
  markDirty();
}

static void wakeScreen() {
  scr = SCR_MAIN;
  M5Dial.Display.setBrightness(brightPct * 255 / 100);
  markDirty();
}

static void sleepScreen() {
  scr = SCR_OFF;
  M5Dial.Display.setBrightness(0);
}

// =========================== Preferences ===================================

static void loadPrefs() {
  prefs.begin("dialfan", true);
  fansOn    = prefs.getBool("on",      false);
  inPct     = constrain(prefs.getInt("inPct",  DEFAULT_INSIDE),  0, 100);
  outPct    = constrain(prefs.getInt("outPct", DEFAULT_OUTSIDE), 0, 100);
  int g     = prefs.getInt("selGrp",  GRP_ALL);
  selGrp    = (FanGroup)constrain(g, 0, 2);
  brightPct = constrain(prefs.getInt("bright", DEFAULT_BRIGHT),
                         BRIGHTNESS_MIN, BRIGHTNESS_MAX);
  prefs.end();
}

static void savePrefs() {
  prefs.begin("dialfan", false);
  prefs.putBool("on",     fansOn);
  prefs.putInt("inPct",   inPct);
  prefs.putInt("outPct",  outPct);
  prefs.putInt("selGrp",  (int)selGrp);
  prefs.putInt("bright",  brightPct);
  prefs.end();
}

// =========================== Colour helpers ================================

static uint16_t hsvTo565(float h, float s, float v) {
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

// Maps 0.0 (cold/slow) → blue to 1.0 (hot/fast) → red
static uint16_t speedColor(float t) {
  t = constrain(t, 0.0f, 1.0f);
  float hue = 220.0f * (1.0f - t);
  return hsvTo565(hue, 1.0f, 0.85f + 0.15f * t);
}

// =========================== Drawing helpers ===============================

static void drawTick(float angleDeg, int16_t rIn, int16_t rOut, uint16_t col) {
  float rad = angleDeg * PI / 180.0f;
  float s = sinf(rad), c = cosf(rad);
  canvas.drawLine(CX + (int)(rIn  * s), CY - (int)(rIn  * c),
                  CX + (int)(rOut * s), CY - (int)(rOut * c), col);
}

static void drawIndicatorDot(int pct, int16_t r1, int16_t r2) {
  float angle = (ARC_START + ARC_SWEEP * pct / 100.0f) * PI / 180.0f;
  float midR = (r1 + r2) / 2.0f;
  int x = CX + (int)(midR * sinf(angle));
  int y = CY - (int)(midR * cosf(angle));
  canvas.fillCircle(x, y, 5, speedColor(pct / 100.0f));
  canvas.drawCircle(x, y, 5, COL_DOT_EDGE);
}

static void drawGradientArc(int16_t r1, int16_t r2, int pct,
                            bool selected, bool systemOn) {
  uint16_t trackCol = !systemOn ? COL_TRACK_OFF
                    : selected  ? COL_TRACK_SEL
                                : COL_TRACK;
  canvas.fillArc(CX, CY, r1, r2, ARC_START, ARC_START + ARC_SWEEP, trackCol);

  if (systemOn && pct > 0) {
    float sweep = ARC_SWEEP * pct / 100.0f;
    const float seg = 3.0f;
    int count = (int)ceilf(sweep / seg);
    for (int i = 0; i < count; i++) {
      float a0 = ARC_START + i * seg;
      float a1 = ARC_START + fminf((i + 1) * seg, sweep);
      float t = (a0 - ARC_START) / ARC_SWEEP;
      canvas.fillArc(CX, CY, r1, r2, a0, a1, speedColor(t));
    }
    drawIndicatorDot(pct, r1, r2);
  }

  if (selected && systemOn) {
    canvas.drawArc(CX, CY, r1 + 1, r1, ARC_START,
                   ARC_START + ARC_SWEEP, COL_SEL_RING);
  }
}

// =========================== Screen drawing ================================

static void drawMainScreen() {
  bool inSel  = (selGrp == GRP_INSIDE  || selGrp == GRP_ALL);
  bool outSel = (selGrp == GRP_OUTSIDE || selGrp == GRP_ALL);

  drawGradientArc(OUT_R1, OUT_R2, fansOn ? outPct : 0, outSel, fansOn);
  drawGradientArc(IN_R1,  IN_R2,  fansOn ? inPct  : 0, inSel,  fansOn);

  for (int i = 0; i <= 4; i++) {
    float angle = ARC_START + ARC_SWEEP * i / 4.0f;
    drawTick(angle, OUT_R1 + 1, OUT_R1 + 5, COL_TICK);
  }

  canvas.setTextDatum(middle_center);

  if (!fansOn) {
    canvas.setFont(&fonts::FreeSansBold18pt7b);
    canvas.setTextColor(COL_OFF);
    canvas.drawString("OFF", CX, CY - 5);

    canvas.setFont(&fonts::FreeSansBold9pt7b);
    canvas.setTextSize(0.7f);
    canvas.setTextColor(COL_LABEL);
    canvas.drawString("press to start", CX, CY + 28);
    canvas.setTextSize(1.0f);
    return;
  }

  // Group label
  canvas.setFont(&fonts::FreeSansBold9pt7b);
  canvas.setTextColor(COL_ACCENT);
  canvas.drawString(GRP_NAMES[selGrp], CX, CY - 30);

  // Main percentage for the selected group
  int dispPct;
  if      (selGrp == GRP_INSIDE)  dispPct = inPct;
  else if (selGrp == GRP_OUTSIDE) dispPct = outPct;
  else                             dispPct = (inPct + outPct + 1) / 2;

  char buf[8];
  snprintf(buf, sizeof(buf), "%d%%", dispPct);
  canvas.setFont(&fonts::FreeSansBold24pt7b);
  canvas.setTextColor(COL_TEXT);
  canvas.drawString(buf, CX, CY + 8);

  // Detail line showing both fan group values
  if (selGrp == GRP_ALL || inPct != outPct) {
    char detail[32];
    snprintf(detail, sizeof(detail), "IN %d%%   OUT %d%%", inPct, outPct);
    canvas.setFont(&fonts::FreeSansBold9pt7b);
    canvas.setTextSize(0.65f);
    canvas.setTextColor(COL_LABEL);
    canvas.drawString(detail, CX, CY + 40);
    canvas.setTextSize(1.0f);
  }
}

static void drawBrightnessScreen() {
  canvas.fillArc(CX, CY, OUT_R1, IN_R2, ARC_START,
                 ARC_START + ARC_SWEEP, COL_TRACK);
  if (brightPct > 0) {
    float sweep = ARC_SWEEP * brightPct / 100.0f;
    canvas.fillArc(CX, CY, OUT_R1, IN_R2, ARC_START,
                   ARC_START + sweep, COL_AMBER);
  }

  for (int i = 0; i <= 4; i++) {
    float angle = ARC_START + ARC_SWEEP * i / 4.0f;
    drawTick(angle, OUT_R1 + 1, OUT_R1 + 5, COL_TICK);
  }

  canvas.setTextDatum(middle_center);

  canvas.setFont(&fonts::FreeSansBold9pt7b);
  canvas.setTextColor(COL_AMBER);
  canvas.drawString("BRIGHTNESS", CX, CY - 25);

  char buf[8];
  snprintf(buf, sizeof(buf), "%d%%", brightPct);
  canvas.setFont(&fonts::FreeSansBold24pt7b);
  canvas.setTextColor(COL_TEXT);
  canvas.drawString(buf, CX, CY + 12);
}
