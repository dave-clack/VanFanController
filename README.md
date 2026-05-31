# VanFanController

PWM fan controller for a campervan, driving a [Noctua NV-FH2](https://noctua.at/en/nv-fh2) 8-channel fan hub from an ESP32-S3 with temperature-based auto and manual modes.

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
