# ME424 Wokwi Circuit Model

This folder contains a Wokwi-importable visual wiring model for your control circuit:

- `diagram.json`

## What this model includes

- Main ESP32 (`main`) and Sensor ESP32 (`sensor`)
- UART link between boards (GPIO19/18 pair represented via UART1 TX/RX symbols)
- Sensor limit inputs:
  - Stage 1 CW limit -> GPIO13
  - Stage 2 limit -> GPIO33
  - Stage 3 limit -> GPIO32
  - Stage 4 limit -> GPIO25
  - Stage 5 Hall input -> GPIO14
- Tool outputs on sensor board:
  - MAG -> GPIO18
  - VAC -> GPIO19
  - SAW MOSFET gate -> GPIO21
  - SAW PWM -> GPIO22
- Tool ID / probe analog model:
  - ADC34 with 10k divider + tool resistor
  - ADC35 shown tied to a potentiometer signal
- Main board motor control indicators:
  - STEP pins: 25, 33, 32, 27, 26, 4
  - DIR pins: 13, 14, 22, 21, 23, 15

## Upload / open online

1. Go to [Wokwi ESP32 New Project](https://wokwi.com/projects/new/esp32).
2. Open `diagram.json` in this folder.
3. Copy all content and paste it into Wokwi's `diagram.json`.
4. Click **Save**.

## Notes

- This is a **visual/logic wiring model** for documentation and interaction testing.
- It is not an exact electrical model of TB6600 drivers, power rails, or saw power electronics.
- If you want, I can generate a second version with:
  - explicit virtual buttons for common commands,
  - a clearer grouped layout for screenshots,
  - or a dedicated variant matching `main_board_sensor_mosfet`.
