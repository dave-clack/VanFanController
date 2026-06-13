# VanFanController

PWM fan controller for a campervan, driving [Noctua NV-FH2](https://noctua.at/en/nv-fh2) fan hubs from ESP32-S3 boards. Two independent apps for two different hardware setups:

| App | Board | Features |
|-----|-------|----------|
| [`VanFanController.ino`](VanFanController.ino) | Adafruit ESP32-S3 Reverse TFT Feather | Single PWM output, temp/humidity sensor, auto & manual modes |
| [`M5DialFanController/`](M5DialFanController/) | M5Stack Dial 1.1 | Dual PWM output (inside/outside fans), rotary encoder, round touch display |

---

# VanFanController (Reverse TFT Feather)

## Hardware

| Component | Part |
|-----------|------|
| MCU + Display | [Adafruit ESP32-S3 Reverse TFT Feather (#5691)](https://www.adafruit.com/product/5691) |
| Temp/Humidity Sensor | [Adafruit SHT41 (#5776)](https://www.adafruit.com/product/5776) via STEMMA QT |
| Fan Hub | [Noctua NV-FH2](https://noctua.at/en/nv-fh2) 8-channel PWM hub |
| Power | 12V leisure battery -> 12V-to-5V buck converter -> Feather USB pin |

## Wiring

### NV-FH2 4-pin input cable to Feather

| Wire   | Fan Pin | Signal | Feather Pin |
|--------|---------|--------|-------------|
| Blue   | 4       | PWM    | A0          |
| Yellow | 3       | Tacho  | A1 (optional) |
| Black  | 1       | GND    | GND         |
| Red    | 2       | +12V   | **Not connected** |

The NV-FH2 gets 12V power from its own SATA connector — do not connect the red wire to the Feather.

### Buck converter to Feather

| Buck Output | Feather Pin |
|-------------|-------------|
| 5V          | USB         |
| GND         | GND         |

Do not connect USB-C and the buck converter simultaneously without a protection diode.

## Modes

- **OFF** — fans off
- **MANUAL** — set fan speed in fixed percentage steps (top/bottom buttons)
- **AUTO** — set a target temperature; fans ramp linearly from off to 100% as cabin temp rises above target + dead zone

## Buttons

| Screen State | Top (short) | Middle (short) | Middle (long) | Bottom (short) | Bottom (long) |
|---|---|---|---|---|---|
| Normal | +setpoint | cycle mode | brightness adjust | -setpoint | toggle RPM overlay |
| Brightness | brighter | exit to normal | — | dimmer | jump to 0% |
| Screen Off | wake | wake | wake | wake | wake |

## Building

Requires [Arduino IDE](https://www.arduino.cc/en/software) or [arduino-cli](https://arduino.github.io/arduino-cli/).

**Board:** Adafruit Feather ESP32-S3 Reverse TFT (via ESP32 by Espressif v3.x board package)

**Libraries** (install via Library Manager):
- Adafruit GFX
- Adafruit ST7735 and ST7789
- Adafruit SHT4x
- Adafruit BusIO
- Adafruit Unified Sensor

**Compile and upload with arduino-cli:**
```bash
arduino-cli compile --fqbn esp32:esp32:adafruit_feather_esp32s3_reversetft .
arduino-cli upload  --fqbn esp32:esp32:adafruit_feather_esp32s3_reversetft --port /dev/cu.usbmodem1101 .
```

## Persistence

Mode, target temperature, manual fan percentage, brightness, and RPM overlay state are saved to NVS flash on change (debounced 1s) and restored at boot.

---

# M5DialFanController (M5Stack Dial 1.1)

Dual-channel fan speed controller with a round touchscreen UI and rotary encoder.

## Hardware

| Component | Part |
|-----------|------|
| MCU + Display | [M5Stack Dial 1.1](https://shop.m5stack.com/products/m5stack-dial-v1-1) (ESP32-S3, 1.28" 240×240 round touch, rotary encoder) |
| Fan Hubs | Noctua NV-FH2 × 2 (one per fan group) |
| Power | 12V leisure battery → 12V-to-5V buck converter → M5Dial USB or DC input (6–36V) |

## Wiring

### PORT.B (HY2.0-4P connector) to fan hubs

| PORT.B Pin | Signal | Fan Hub Connection |
|------------|--------|--------------------|
| G1         | PWM    | Inside fan hub input (pin 4, blue wire) |
| G2         | PWM    | Outside fan hub input (pin 4, blue wire) |
| GND        | GND    | Fan hub GND (pin 1, black wire) |
| 5V         | —      | Not connected (hubs powered via SATA) |

### Temperature sensor (future)

PORT.A provides I2C (G13 SCL / G15 SDA) on a separate bus from the internal peripherals. Connect an SHT41 sensor using a Grove-to-STEMMA-QT adapter cable.

## Display

Two concentric gradient arcs on a dark theme, spanning from 7 o'clock (0%) to 5 o'clock (100%):

- **Outer ring** — outside fan speed
- **Inner ring** — inside fan speed
- Active ring(s) have a bright colour-matched outline
- Centre shows the current mode, percentage, and both fan values
- 10 selectable colour palettes (e.g. Blue→Cyan / Green→Yellow, Pink→Purple / Orange→Red)

## Controls

| Input | Action |
|-------|--------|
| Press dial | Cycle mode: OFF → INSIDE → OUTSIDE → ALL → OFF |
| Turn dial | Adjust fan speed of selected group |
| Tap screen | Cycle: main → brightness → palette → main |

### Brightness mode

| Input | Action |
|-------|--------|
| Turn dial | Adjust brightness (step ladder: 100, 90, …, 10, 5, 2, 1) |
| Tap screen | Switch to palette mode |
| Press dial | Return to main screen |
| 5-second timeout | Return to main screen |

### Palette mode

| Input | Action |
|-------|--------|
| Turn dial | Cycle through 10 colour palettes (preview at 100%) |
| Tap screen | Return to main screen |
| Press dial | Return to main screen |
| 5-second timeout | Return to main screen |

### OFF mode

When the device is off, the screen goes dark to save power. Any input (touch, encoder, button) wakes the screen to show the OFF display. From there, tapping the screen or pressing the button turns the device on (INSIDE mode). Brightness is set to at least 25% on turn-on.

## Building

**Board:** M5Stack Dial (or ESP32-S3) via ESP32 by Espressif board package

**Library** (install via Library Manager): M5Dial (includes M5Unified + M5GFX)

**Compile and upload with arduino-cli:**
```bash
arduino-cli compile --fqbn esp32:esp32:m5stack_dial M5DialFanController/
arduino-cli upload  --fqbn esp32:esp32:m5stack_dial --port /dev/cu.usbmodem* M5DialFanController/
```

## Persistence

Mode, inside/outside speeds, brightness, and palette selection are saved to NVS flash on change (debounced 1s) and restored at boot.
