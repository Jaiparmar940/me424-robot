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
│  - Wi‑Fi     │   UART1    ┌──────────────┐
│  - Steppers  │ ──────────▸│              │
│  - Commands  │            │ Sensor Board │   (limit switches, tool ID)
│              │◂────────── │              │
└──────────────┘   115200   └──────────────┘
```

| Board | Role |
|---|---|
| **Main Board** | Master controller. USB serial, Wi‑Fi (HTTP UI + WebSocket + OTA), drives 6 stepper motors via TB6600 drivers, routes commands to other boards over UART. |
| **MOSFET Board** | Receives commands from Main Board over UART. Drives 3 MOSFET outputs for electromagnet, vacuum, and circular saw. Sends ACK responses. |
| **Sensor Board** | Reads three limit switches plus Stage 5 Hall-at-zero (GPIO14 / HW-477) and reports state changes to Main Board over UART. |

## Project Structure

```
ME424_Robot/
├── platformio.ini              Multi-environment build config
├── src/
│   ├── main_board/main.cpp     Main board firmware
│   ├── mosfet_board/main.cpp   MOSFET slave board firmware
│   └── sensor_board/main.cpp   Sensor board firmware
├── include/                    Shared headers (empty, for future use)
├── lib/                        Shared libraries (empty, for future use)
└── .gitignore
```

---

## Pin Assignments

### Main Board

#### Stepper Motor Outputs (to TB6600 drivers)

| Controller | Stage Mapping | STEP Pin | DIR Pin | Direction Inverted |
|---|---|---|---|---|
| 1 | Stage 2 Right | GPIO 25 | GPIO 13 | No |
| 2 | Stage 3 | GPIO 33 | GPIO 14 | No |
| 3 | Stage 4 | GPIO 32 | GPIO 22 | No |
| 4 | Stage 2 Left | GPIO 27 | GPIO 21 | **Yes** |
| 5 | Stage 5 | GPIO 26 | GPIO 23 | No |
| 6 | Stage 1 (turntable) | GPIO 4 | GPIO 15 | No |

Stage 2 uses two motors (controllers 1 and 4) driven in sync. Controller 4 has its direction inverted so both motors move the same physical direction.

#### UART Connections

| Bus | Function | Main Board Pin | Remote Board Pin | Baud |
|---|---|---|---|---|
| UART2 | To MOSFET board | RX = GPIO 16, TX = GPIO 17 | MOSFET RX = GPIO 25, TX = GPIO 26 | 115200 |
| UART1 | To Sensor board | RX = GPIO 18, TX = GPIO 19 | Sensor TX = GPIO 27, RX = GPIO 26 | 115200 |
| UART0 | USB Serial (debug) | Default (GPIO 1 / 3) | — | 115200 |

#### Other

| Function | Details |
|---|---|
| Wi‑Fi | Station mode: OTA updates, HTTP dashboard on port 80, WebSocket commands on 81, optional TCP 3333 |

---

### MOSFET Board

| Function | GPIO | Mode | Notes |
|---|---|---|---|
| Electromagnet output | 18 | OUTPUT | Active HIGH |
| Vacuum output | 19 | OUTPUT | Active HIGH |
| Saw output | 21 | OUTPUT | Active HIGH |
| UART RX (from Main TX) | 25 | UART1 RX | Cross-wired to Main GPIO 17 |
| UART TX (to Main RX) | 26 | UART1 TX | Cross-wired to Main GPIO 16 |
| USB Serial (debug) | Default | UART0 | 115200 baud |

All outputs default to LOW (off) on boot. The board sends `SLAVE READY` to the main board on startup.

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

## UART Wiring Diagram

```
Main Board                    MOSFET Board
──────────                    ────────────
GPIO 16 (RX2)  ◂──────────  GPIO 26 (TX1)
GPIO 17 (TX2)  ──────────▸  GPIO 25 (RX1)
GND            ──────────── GND

Main Board                    Sensor Board
──────────                    ────────────
GPIO 18 (RX1)  ◂──────────  GPIO 27 (TX1)
GPIO 19 (TX1)  ──────────▸  GPIO 26 (RX1)
GND            ──────────── GND
```

All UART buses run at **115200 baud, 8N1**.

---

## Command Reference

Commands are sent to the Main Board over **USB serial**, **WebSocket** (same line protocol as serial), or **TCP 3333**. Each command is case-insensitive and terminated by a newline (`\n`). Responses are echoed to USB and any active network clients.

### Stage Commands

| Command | Description |
|---|---|
| `s1cw <steps>` | Stage 1 (turntable) clockwise |
| `s1ccw <steps>` | Stage 1 (turntable) counter-clockwise |
| `s2up <steps>` | Stage 2 up (controllers 1 + 4 in sync, forward) |
| `s2down <steps>` | Stage 2 down (controllers 1 + 4 in sync, reverse) |
| `s3up <steps>` | Stage 3 up |
| `s3down <steps>` | Stage 3 down |
| `s4up <steps>` | Stage 4 up |
| `s4down <steps>` | Stage 4 down |
| `s5cw <steps>` | Stage 5 clockwise |
| `s5ccw <steps>` | Stage 5 counter-clockwise |

### Raw Controller Commands

| Command | Description |
|---|---|
| `c1f <steps>` / `c1r <steps>` | Controller 1 forward / reverse (Stage 2 Right) |
| `c2f <steps>` / `c2r <steps>` | Controller 2 forward / reverse (Stage 3) |
| `c3f <steps>` / `c3r <steps>` | Controller 3 forward / reverse (Stage 4) |
| `c4f <steps>` / `c4r <steps>` | Controller 4 forward / reverse (Stage 2 Left) |
| `c5f <steps>` / `c5r <steps>` | Controller 5 forward / reverse (Stage 5) |
| `c6f <steps>` / `c6r <steps>` | Controller 6 forward / reverse (Stage 1 / turntable) |

### MOSFET / Tool Commands

These are forwarded from the Main Board to the MOSFET Board over UART.

| Command | Forwarded As | MOSFET Board Response |
|---|---|---|
| `mag on` / `mag off` | `MAG ON` / `MAG OFF` | `ACK MAG ON` / `ACK MAG OFF` |
| `vac on` / `vac off` | `VAC ON` / `VAC OFF` | `ACK VAC ON` / `ACK VAC OFF` |
| `saw on` / `saw off` | `SAW ON` / `SAW OFF` | `ACK SAW ON` / `ACK SAW OFF` |
| `alloff` | `ALL OFF` | `ACK ALL OFF` |
| `mstatus` | `STATUS` | `STATUS MAG=ON/OFF VAC=ON/OFF SAW=ON/OFF` |

### Sequence Commands

Commands are separated by `,`. Whitespace around commas is ignored.

| Command | Description |
|---|---|
| `seq <cmd1>, <cmd2>, ...` | Execute commands one at a time in order. Any command type is valid (motor, MOSFET, utility). |
| `par <cmd1>, <cmd2>, ...` | Execute motor commands simultaneously. All specified motors are stepped on the same clock tick. Motor/stage commands only — MOSFET commands are not supported in `par`. |
| `syncabs t1 t2 t3 t4 t5 t6 maxSps rampSteps` | Execute a synchronized **absolute** 6-axis move. Each axis reaches its target at the same time, speed is capped by `maxSps` (steps/s), and accel/decel ramping is applied using `rampSteps`. |
| `qclear` | Clear onboard queue |
| `qadd pose t1 t2 t3 t4 t5 t6 maxSps rampSteps` | Append a synchronized absolute move segment to onboard queue |
| `qadd delay ms` | Append delay segment to onboard queue |
| `qadd cmd <firmware command>` | Append command segment (e.g. `vac on`, `alloff`) |
| `qlist` | Print current queue items |
| `qrun` | Execute queued items on firmware (true onboard playlist execution) |
| `qstop` | Request stop for running queue |
| `qstatus` | Print queue count and stop state |

**Examples:**
```
seq s2up 400, s3down 200, mag on
par s2up 400, s5cw 200
par s3down 200, s4down 300, s1cw 100
seq par s3down 400, s4down 400, s2up 200, mag on
syncabs 0 1000 -500 0 200 0 1200 300
qclear
qadd pose 0 400 0 0 0 0 900 150
qadd delay 500
qadd cmd vac on
qadd pose 0 0 0 0 0 0 900 150
qadd cmd vac off
qrun
```

In `par`, motors with different step counts run until each individually finishes — longer moves continue while shorter ones stop. Limits are checked per-motor on every step.

### Utility Commands

| Command | Description |
|---|---|
| `speed <us>` | Set stepper pulse delay in microseconds (minimum 100) |
| `where` | Print the tracked current position for all 6 controllers |
| `setpos p1 p2 p3 p4 p5 p6` | Overwrite the tracked position state (useful after manual zeroing/homing) |
| `estop` | Latch emergency stop immediately (also during active motion/queue). Motion commands are blocked until cleared. Also sends `ALL OFF` to MOSFET board. |
| `estop clear` | Clear the e-stop latch to allow motion again |
| `estop status` | Print `ESTOP=1` or `ESTOP=0` |
| `limits` | Print last-known limit switch and Stage 5 Hall (`S5H`) states |
| `home s1` | Soft-zero turntable: set tracked C6 position to 0 at the current pose |
| `home s2` | Drive Stage 2 toward its limit, then zero C1/C4 at the limit |
| `home s3` | Drive Stage 3 toward its limit, then zero C2 |
| `home s4` | Drive Stage 4 toward its limit, then zero C3 |
| `home s5` | Jog Stage 5 until Hall at zero (`S5H=0`), then zero C5 |
| `help` | Print command menu |

---

## Limit Switch Safety

The sensor board monitors three limit switches plus a Stage 5 Hall line (3144-style sensor through an HW-477 comparator on **GPIO14**; homing-only, not a motion limit) and sends state changes to the main board using this protocol:

```
LIM S2=1 S3=0 S4=0 S5H=0
```

For **S2–S4**, `1` = limit switch triggered (pressed), `0` = open. For **S5H** the convention is reversed: **`0` = at Hall zero position**, `1` = away from zero. Adjust the HW-477 threshold pot. Sensor firmware defaults to **`S5_HALL_AT_ZERO_ACTIVE_HIGH = false`** (pin **LOW** = at zero); set to `true` if your comparator output is the opposite.

### Behavior

The main board uses direction-aware limit logic. Limits only block motion in the **downward** direction for a given stage — upward motion is always allowed even if the limit is triggered. The sensor UART is polled mid-step so limits are checked during long moves and will halt motion immediately.

If a limit is already active when a command is received, the command is rejected with an `ABORT` message. If a limit triggers mid-move, the motors stop and a `STOP` message is sent.

### Direction Mapping

| Stage | "Down" Direction | Limit Blocks |
|---|---|---|
| Stage 1 | — | No limit switch |
| Stage 2 | `s2down` (forward = false) | Downward motion only |
| Stage 3 | `s3down` (forward = true) | Downward motion only |
| Stage 4 | `s4down` (forward = true) | Downward motion only |
| Stage 5 | — | No limit switch (Hall `S5H` on sensor board is for **homing only**, not blocking motion) |

### Limit-to-Motor Mapping

| Limit Switch | Motors Affected |
|---|---|
| Stage 2 | Controllers 1 + 4 (stopped together) |
| Stage 3 | Controller 2 |
| Stage 4 | Controller 3 |

---

## Wi‑Fi usage (replaces Bluetooth)

Bluetooth Classic has been removed to save flash/RAM. Use one of the following:

| Channel | Purpose |
|---|---|
| USB serial | 115200 baud, same as before |
| Browser | Open `http://me424-main.local/` (or the main board IP) for the hosted web app; it connects over WebSocket port 81 |
| TCP | `nc me424-main.local 3333` for raw line commands |
| OTA | PlatformIO `*_ota` environments upload firmware over the network |

### Serial terminal settings (USB)

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
pio run -e mosfet_board
pio run -e sensor_board

# Upload to a specific board (auto-detects USB port)
pio run -e main_board -t upload
pio run -e mosfet_board -t upload
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

### WiFi / OTA Upload (All Boards)

Bluetooth transport on the main board has been replaced with WiFi command + OTA support.  
All three boards now support Arduino OTA when WiFi credentials are compiled in.

1. Set WiFi credentials in `platformio.ini` under `[env]` (`build_flags`: `WIFI_SSID`, `WIFI_PASSWORD`). Each board env merges these via `${env.build_flags}`.
2. First flash each board once over USB:

```bash
pio run -e main_board -t upload
pio run -e mosfet_board -t upload
pio run -e sensor_board -t upload
```

3. Then use OTA environments:

```bash
pio run -e main_board_ota -t upload
pio run -e mosfet_board_ota -t upload
pio run -e sensor_board_ota -t upload
```

Default OTA hostnames are:
- `me424-main.local`
- `me424-mosfet.local`
- `me424-sensor.local`

### Wireless web app (main board)

The main board serves the dashboard from **LittleFS** at **`http://<ip>/`** (try `http://me424-main.local/`). The same UI talks to the firmware over **WebSocket** port **81** (line-based commands, same as USB serial). Raw TCP on port **3333** still works for `nc` / scripts.

After changing files in `app/`, copy them into `data/` and upload the filesystem (USB or OTA):

```bash
cp app/index.html app/app.js app/styles.css data/
pio run -e main_board -t uploadfs
# or, after OTA works:
pio run -e main_board_ota -t uploadfs
```

Open the UI in a browser on the same Wi‑Fi network; the page auto-connects over Web‑Fi. Use `http://localhost:8080/app/` (or similar) with **Connect (USB Serial)** for local dev, or add **`?usb=1`** to the robot URL to force USB from a desktop.

### Setting Upload Ports

If you have multiple ESP32 boards connected simultaneously, uncomment and set the `upload_port` lines in `platformio.ini`:

```ini
[env:main_board]
build_src_filter = -<*> +<main_board/>
upload_port = /dev/cu.usbserial-0001

[env:mosfet_board]
build_src_filter = -<*> +<mosfet_board/>
upload_port = /dev/cu.usbserial-0002

[env:sensor_board]
build_src_filter = -<*> +<sensor_board/>
upload_port = /dev/cu.usbserial-0003
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
2. **MOSFET board** — starts listening for commands, all outputs default off
3. **Main board** — expects the other two boards to already be running

---

## Web App (8 Requirements UI)

A browser app is included at `app/` and uses Web Serial to talk directly to the main ESP.

### Run the App

1. Start a local static server from repo root:
   - `python3 -m http.server 8080`
2. Open [http://localhost:8080/app/](http://localhost:8080/app/)
3. Click **Connect (Web Serial)** and select your ESP32 serial port.

Use Chrome or Edge desktop (Web Serial is not supported in Safari/Firefox).

### Requirement Coverage

1. **Manual jog with granularity**
   - `Manual Control` section
   - Set `Step Size`
   - Use `+ Jog` / `- Jog` per motor

2. **Record all 6 positions in one state**
   - `Record Current Pose` captures all 6 tracked positions

3. **Add delays between states**
   - Set `Delay (ms)` and click `Add Delay`

4. **Append more recorded positions**
   - Record additional poses; list appends in order

5. **Toggle MOSFET functions in same list**
   - Choose `MOSFET Action`, click `Add MOSFET Action`

6. **Replay with synchronized finish + ramps + max speed**
   - Sequence playback uses firmware command:
     - `syncabs t1 t2 t3 t4 t5 t6 maxSps rampSteps`
   - `Playback maxSps` and `Playback rampSteps` control speed/ramp
   - Axis motion is synchronized so all active axes finish together

7. **Zero / home from buttons**
   - `Set Current as Zero (All)` → soft-zero all tracked positions (`setpos …`)
   - `Zero This Axis` on each axis → soft-zero that controller only
   - `Auto home (home sN)` under each stage → runs the matching firmware homing command (`home s1` … `home s5`)
   - `Home Stages 2–4 (limits)` → runs `home s2`, then `home s3`, then `home s4` in order

8. **Save commands locally**
   - `Save Local` / `Load Local` uses browser `localStorage`
   - `Export JSON` / `Import JSON` for file-based reuse

### App Data Model

Sequence items are one of:
- `pose`: six-axis absolute positions
- `delay`: milliseconds
- `mosfet`: command string (`mag on`, `vac off`, etc.)

---

## Inter-Board Protocols

### Sensor Board (Sensor <-> Main)

#### Messages Sent (Sensor -> Main)

| Message | Meaning |
|---|---|
| `SENSOR READY` | Boot complete |
| `LIM S2=<0\|1> S3=<0\|1> S4=<0\|1> S5H=<0\|1>` | S2–S4: `1` = limit hit; **S5H: `0` = at Hall zero**, `1` = away (sent on change and at boot) |
| `ERR UNKNOWN_CMD` | Unrecognized command received |

#### Commands Accepted (Main -> Sensor)

| Command | Response |
|---|---|
| `STATUS` | Sends current `LIM ...` message |

### MOSFET Board (MOSFET <-> Main)

#### Messages Sent (MOSFET -> Main)

| Message | Meaning |
|---|---|
| `SLAVE READY` | Boot complete |
| `ACK MAG ON` / `ACK MAG OFF` | Electromagnet toggled |
| `ACK VAC ON` / `ACK VAC OFF` | Vacuum toggled |
| `ACK SAW ON` / `ACK SAW OFF` | Saw toggled |
| `ACK ALL OFF` | All outputs turned off |
| `STATUS MAG=<ON\|OFF> VAC=<ON\|OFF> SAW=<ON\|OFF>` | Current output states |
| `ERR UNKNOWN_CMD` | Unrecognized command received |

#### Commands Accepted (Main -> MOSFET)

| Command | Response |
|---|---|
| `MAG ON` / `MAG OFF` | `ACK MAG ON` / `ACK MAG OFF` |
| `VAC ON` / `VAC OFF` | `ACK VAC ON` / `ACK VAC OFF` |
| `SAW ON` / `SAW OFF` | `ACK SAW ON` / `ACK SAW OFF` |
| `ALL OFF` | `ACK ALL OFF` |
| `STATUS` | `STATUS MAG=... VAC=... SAW=...` |

---

## Future Work

- [ ] Add tool identification via ADC voltage divider on the sensor board
- [ ] Add linear probe (potentiometer) reading
- [ ] Add debouncing / filtering to limit switch reads
- [ ] Add shared protocol header in `lib/` for message definitions
