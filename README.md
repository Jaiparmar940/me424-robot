# ME424 Robot — Multi-Board ESP32 Firmware

Firmware for a 5-DOF robotic arm with an interchangeable tool head system. Three ESP32 dev boards communicate over UART to coordinate stepper motors, limit switches, and tool attachments (electromagnet, vacuum, circular saw, linear probe, etc.).

The wrist uses an 8-pin magnetic connector to hot-swap tools.

## System Architecture

```
┌──────────────┐   UART2    ┌──────────────┐
│              │ ──────────▸│              │
│  Main Board  │            │ MOSFET Board │   (tool power switching)
│              │◂────────── │              │
│              │   115200   └──────────────┘
│              │
│  - Bluetooth │   UART1    ┌──────────────┐
│  - Steppers  │ ──────────▸│              │
│  - Commands  │            │ Sensor Board │   (limit switches, tool ID)
│              │◂────────── │              │
└──────────────┘   115200   └──────────────┘
```

| Board | Role |
|---|---|
| **Main Board** | Master controller. Bluetooth serial interface, drives 5 stepper motors via TB6600 drivers, routes commands to other boards over UART. |
| **MOSFET Board** | Receives commands from Main Board to switch high-current outputs (electromagnet, vacuum, saw). Firmware for this board is not yet implemented — only the Main Board side that sends commands to it exists. |
| **Sensor Board** | Reads 3 limit switches and reports state changes to Main Board over UART. |

## Project Structure

```
ME424_Robot/
├── platformio.ini              Multi-environment build config
├── src/
│   ├── main_board/main.cpp     Main board firmware (v1 — s2up/s2down commands)
│   ├── mosfet_board/main.cpp   Main board firmware (v2 — s2f/s2r commands, lstatus)
│   └── sensor_board/main.cpp   Sensor board firmware
├── include/                    Shared headers (empty, for future use)
├── lib/                        Shared libraries (empty, for future use)
└── .gitignore
```

> **Note:** `main_board` and `mosfet_board` are two variants of the master firmware, not two different boards. See [Firmware Variants](#firmware-variants) below.

---

## Pin Assignments

### Main Board (both firmware variants)

#### Stepper Motor Outputs (to TB6600 drivers)

| Controller | Stage Mapping | STEP Pin | DIR Pin | Direction Inverted |
|---|---|---|---|---|
| 1 | Stage 2 Right | GPIO 25 | GPIO 13 | No |
| 2 | Stage 3 | GPIO 33 | GPIO 14 | No |
| 3 | Unassigned | GPIO 32 | GPIO 22 | No |
| 4 | Stage 2 Left | GPIO 27 | GPIO 21 | **Yes** |
| 5 | Stage 5 (v1 only) | GPIO 26 | GPIO 23 | No |

Stage 2 uses two motors (controllers 1 and 4) driven in sync. Controller 4 has its direction inverted so both motors move the same physical direction.

#### UART Connections

| Bus | Function | Main Board Pin | Remote Board Pin | Baud |
|---|---|---|---|---|
| UART2 | To MOSFET board | RX = GPIO 16, TX = GPIO 17 | MOSFET RX / TX | 115200 |
| UART1 | To Sensor board | RX = GPIO 18, TX = GPIO 19 | Sensor TX = GPIO 27, RX = GPIO 26 | 115200 |
| UART0 | USB Serial (debug) | Default (GPIO 1 / 3) | — | 115200 |

#### Other

| Function | Details |
|---|---|
| Bluetooth Classic | Advertises as `ESP32_STAGE_CTRL` at 115200 baud |

---

### Sensor Board

| Function | GPIO | Mode | Notes |
|---|---|---|---|
| Stage 2 Limit Switch | 33 | INPUT_PULLUP | Active LOW (switch wired to GND) |
| Stage 3 Limit Switch | 32 | INPUT_PULLUP | Active LOW |
| Stage 4 Limit Switch | 25 | INPUT_PULLUP | Active LOW |
| UART TX (to Main RX) | 27 | UART1 TX | Cross-wired to Main GPIO 18 |
| UART RX (from Main TX) | 26 | UART1 RX | Cross-wired to Main GPIO 19 |
| USB Serial (debug) | Default | UART0 | 115200 baud |

---

### MOSFET Board

The MOSFET board receives text commands over UART from the Main Board and toggles high-current outputs. **Its firmware is not yet written.** The Main Board sends it these commands:

| Command | Expected Action |
|---|---|
| `MAG ON` / `MAG OFF` | Toggle electromagnet |
| `VAC ON` / `VAC OFF` | Toggle vacuum |
| `SAW ON` / `SAW OFF` | Toggle circular saw |
| `ALL OFF` | All outputs off |
| `STATUS` | Report current output states |

---

## UART Wiring Diagram

```
Main Board                    Sensor Board
──────────                    ────────────
GPIO 18 (RX1)  ◂──────────  GPIO 27 (TX1)
GPIO 19 (TX1)  ──────────▸  GPIO 26 (RX1)
GND            ──────────── GND

Main Board                    MOSFET Board
──────────                    ────────────
GPIO 16 (RX2)  ◂──────────  TX
GPIO 17 (TX2)  ──────────▸  RX
GND            ──────────── GND
```

All UART buses run at **115200 baud, 8N1**.

---

## Firmware Variants

There are two versions of the master firmware in this repo. Both run on the same physical Main Board — pick one.

| Feature | `main_board` (v1) | `mosfet_board` (v2) |
|---|---|---|
| Stage 2 commands | `s2up` / `s2down` | `s2f` / `s2r` |
| Stage 3 commands | `s3up` / `s3down` | `s3f` / `s3r` |
| Stage 5 support | Yes (`s5cw` / `s5ccw`) | No |
| Direction-aware limits | Yes (only blocks downward) | No (blocks any direction if hit) |
| `lstatus` command | No | Yes (requests fresh sensor data) |
| Sensor polling | Inline during stepping (`serviceSensorUART`) | Polled each loop iteration |
| MOSFET UART name | `SlaveSerial` | `MosfetSerial` |

---

## Command Reference

Commands are case-insensitive and can be sent over USB Serial or Bluetooth. Each command is terminated by a newline (`\n`).

### Stage Commands

#### v1 (`main_board`)

| Command | Description |
|---|---|
| `s2up <steps>` | Stage 2 up (both motors, forward) |
| `s2down <steps>` | Stage 2 down (both motors, reverse) |
| `s3up <steps>` | Stage 3 up |
| `s3down <steps>` | Stage 3 down |
| `s5cw <steps>` | Stage 5 clockwise |
| `s5ccw <steps>` | Stage 5 counter-clockwise |

#### v2 (`mosfet_board`)

| Command | Description |
|---|---|
| `s2f <steps>` | Stage 2 forward (both motors) |
| `s2r <steps>` | Stage 2 reverse (both motors) |
| `s3f <steps>` | Stage 3 forward |
| `s3r <steps>` | Stage 3 reverse |

### Raw Controller Commands (both versions)

| Command | Description |
|---|---|
| `c1f <steps>` | Controller 1 forward |
| `c1r <steps>` | Controller 1 reverse |
| `c2f <steps>` | Controller 2 forward |
| `c2r <steps>` | Controller 2 reverse |
| `c3f <steps>` | Controller 3 forward (unassigned motor) |
| `c3r <steps>` | Controller 3 reverse |
| `c4f <steps>` | Controller 4 forward |
| `c4r <steps>` | Controller 4 reverse |
| `c5f <steps>` | Controller 5 forward |
| `c5r <steps>` | Controller 5 reverse |

### MOSFET / Tool Commands (both versions)

| Command | Description |
|---|---|
| `mag on` / `mag off` | Electromagnet on/off |
| `vac on` / `vac off` | Vacuum on/off |
| `saw on` / `saw off` | Circular saw on/off |
| `alloff` | All MOSFET outputs off |
| `mstatus` | Request MOSFET board status |

### Utility Commands

| Command | v1 | v2 | Description |
|---|---|---|---|
| `speed <us>` | Yes | Yes | Set stepper pulse delay in microseconds (min 100) |
| `limits` | Yes | Yes | Print last-known limit switch states |
| `lstatus` | No | Yes | Request fresh status from sensor board |
| `help` | Yes | Yes | Print command menu |

---

## Limit Switch Safety

The sensor board monitors 3 limit switches and sends state changes to the main board using this protocol:

```
LIM S2=1 S3=0 S4=0
```

Where `1` = switch triggered (pressed), `0` = switch open.

### Behavior

- **v1 (`main_board`):** Direction-aware. Only blocks motion in the *downward* direction for a given stage. Upward motion is always allowed even if the limit is triggered. The sensor UART is polled mid-step so limits are respected during long moves.
- **v2 (`mosfet_board`):** Blocks motion in *any* direction when a limit is hit for that stage.
- Both versions abort immediately and report over Serial + Bluetooth when a limit stops motion.

### Limit-to-Stage Mapping

| Limit Switch | Blocks | Motors Affected |
|---|---|---|
| Stage 2 | Stage 2 pair (controllers 1 + 4) | Both stopped together |
| Stage 3 | Stage 3 (controller 2) | Single motor |
| Stage 4 | Monitored but no motor mapped | — |

---

## Bluetooth Usage

The main board advertises as a Bluetooth Classic (SPP) device named **`ESP32_STAGE_CTRL`**.

### Connecting

1. Power on the main board.
2. On your phone/laptop, scan for Bluetooth devices and pair with `ESP32_STAGE_CTRL`.
3. Open a Bluetooth serial terminal app:
   - **Android:** "Serial Bluetooth Terminal" (free on Play Store)
   - **iOS:** Bluetooth Classic SPP is not natively supported on iOS; use a BLE bridge or a laptop
   - **macOS/Linux:** `screen /dev/cu.ESP32_STAGE_CTRL 115200` or any serial terminal
   - **Windows:** Pair the device, note the COM port, open with PuTTY/Tera Term at 115200
4. Send any command from the [Command Reference](#command-reference). Responses are echoed to both Bluetooth and USB Serial simultaneously.

### Terminal Settings

| Setting | Value |
|---|---|
| Baud rate | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Line ending | Newline (`\n`) |

---

## Tool Connector (8-Pin Magnetic)

The wrist connector carries power and signals to interchangeable tools:

| Pin(s) | Function | Direction | Notes |
|---|---|---|---|
| 1, 2 | V+ (tool power) | Arm -> Tool | Paralleled for current capacity |
| 3, 4 | GND | — | Paralleled |
| 5 | BLDC PWM control | Arm -> Tool | PWM signal to on-tool motor driver |
| 6 | BLDC enable / digital control | Arm -> Tool | Enable line |
| 7 | ADC signal (shared) | Tool -> Arm | Probe potentiometer OR tool ID resistor |
| 8 | Spare | — | Reserved for future use |

### Tool Identification

Each tool contains a resistor to GND on pin 7. The arm has a fixed pull-up resistor to 3.3V on the same pin, forming a voltage divider read by the sensor board's ADC. Different resistance values identify different tools.

```
3.3V ── [fixed R on board] ──┬── ADC pin (GPIO on sensor board)
                              │
                    [tool R] ── GND (through connector)
```

> Tool ID and ADC reading are not yet implemented in firmware.

---

## Building and Uploading

### Prerequisites

- [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode) installed in VS Code / Cursor
- ESP32 USB drivers installed (CP2102 or CH340 depending on your dev board)

### Using the PlatformIO IDE (recommended)

1. Open the `ME424_Robot` folder in Cursor/VS Code.
2. PlatformIO will auto-detect `platformio.ini` and index the project.
3. Use the PlatformIO sidebar or the bottom toolbar:
   - Click the **environment switcher** in the status bar to select `main_board`, `mosfet_board`, or `sensor_board`.
   - Click **Build** (checkmark icon) to compile.
   - Click **Upload** (arrow icon) to flash the selected board.
   - Click **Serial Monitor** (plug icon) to open the monitor at 115200 baud.

### Using the PlatformIO CLI

```bash
# Build a specific board
pio run -e main_board
pio run -e sensor_board

# Upload to a specific board (auto-detects USB port)
pio run -e main_board -t upload
pio run -e sensor_board -t upload

# Upload to a specific USB port
pio run -e main_board -t upload --upload-port /dev/cu.usbserial-0001

# Open serial monitor
pio device monitor -e main_board

# Build all environments
pio run

# Clean build artifacts
pio run -t clean
```

### Setting Upload Ports

If you have multiple ESP32 boards connected simultaneously, uncomment and set the `upload_port` lines in `platformio.ini`:

```ini
[env:main_board]
build_src_filter = -<*> +<main_board/>
upload_port = /dev/cu.usbserial-0001

[env:sensor_board]
build_src_filter = -<*> +<sensor_board/>
upload_port = /dev/cu.usbserial-0002
```

To find your port names:

```bash
# macOS / Linux
ls /dev/cu.usb*

# Windows (in PlatformIO terminal)
pio device list
```

### Upload Order

There is no strict order, but a practical sequence is:

1. **Sensor board** — starts reporting limits immediately on boot
2. **Main board** — expects sensor board to already be running
3. **MOSFET board** — receives commands from main board (firmware TBD)

---

## Sensor Board Protocol

### Messages Sent (Sensor -> Main)

| Message | Meaning |
|---|---|
| `SENSOR READY` | Boot complete |
| `LIM S2=<0\|1> S3=<0\|1> S4=<0\|1>` | Limit switch states (sent on change and on request) |
| `ERR UNKNOWN_CMD` | Unrecognized command received |

### Commands Accepted (Main -> Sensor)

| Command | Response |
|---|---|
| `STATUS` | Sends current `LIM ...` message |

---

## MOSFET Board Protocol

### Commands Accepted (Main -> MOSFET)

| Command | Expected Action |
|---|---|
| `MAG ON` / `MAG OFF` | Toggle electromagnet output |
| `VAC ON` / `VAC OFF` | Toggle vacuum output |
| `SAW ON` / `SAW OFF` | Toggle saw output |
| `ALL OFF` | All outputs off |
| `STATUS` | Report output states |

> MOSFET board firmware is not yet implemented. These commands are sent by the main board but there is no receiving firmware in this repo.

---

## Future Work

- [ ] Implement MOSFET board firmware (receive UART commands, drive MOSFET outputs)
- [ ] Add tool identification via ADC voltage divider on the sensor board
- [ ] Add linear probe (potentiometer) reading
- [ ] Implement Stage 1 (turntable) and Stage 4 motor control
- [ ] Add debouncing / filtering to limit switch reads
- [ ] Consolidate the two main board firmware variants into one
- [ ] Add shared protocol header in `lib/` for message definitions
