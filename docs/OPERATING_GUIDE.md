# Operating the robot

The preserved source is `Controller_Python.py`, `master/master.ino`, and `slave/slave.ino`. The final report explains the hardware and historical demonstrations.

## Install and build

Use Python 3.12 and an Xbox-compatible gamepad with the expected Pygame button/axis indices:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

Firmware uses the Espressif Arduino-ESP32 board package, built-in Arduino/WiFi/WiFiUDP/Wire libraries, and Adafruit VL53L0X for the slave. The report identifies the master as an Unexpected Maker FeatherS2 and the slave as an M5Stamp C3. Candidate FQBNs: `esp32:esp32:um_feathers2` and `esp32:esp32:m5stack_stamp_c3`. Confirm the physical board before flashing.

The missing Vive interface was located in the supplied archive at `final project/vive1/vive510.h` and `vive510.cpp`. It is the Penn MEAM510 interface credited to Mark Yim, December 2021. Its source copyright header is retained. It is a separate course dependency, not a newly implemented library or a file covered automatically by this repository's MIT notice. Supply it locally; it is not republished here.

```powershell
arduino-cli core update-index --additional-urls 'https://espressif.github.io/arduino-esp32/package_esp32_index.json'
arduino-cli core install esp32:esp32@3.3.2 --additional-urls 'https://espressif.github.io/arduino-esp32/package_esp32_index.json'
arduino-cli lib install Adafruit_VL53L0X@1.2.5
.\.venv\Scripts\python.exe scripts/stage_firmware.py --vive-source 'C:\path to supplied archive\final project\vive1'
arduino-cli compile --fqbn esp32:esp32:um_feathers2 'outputs/firmware/master'
arduino-cli compile --fqbn esp32:esp32:m5stack_stamp_c3 'outputs/firmware/slave'
# Replace COM_PORT with the physically verified port; compile before upload.
arduino-cli upload --fqbn esp32:esp32:um_feathers2 --port COM_PORT 'outputs/firmware/master'
arduino-cli upload --fqbn esp32:esp32:m5stack_stamp_c3 --port COM_PORT 'outputs/firmware/slave'
```

The presentation update's executed checks and compilation outcome are recorded in [verification](VERIFICATION.md). Compilation is not a hardware test.

## Driver station

Connect the laptop to the robot AP `Xbox_Joy`, verify the gamepad, then run `python Controller_Python.py`. The AP is open in this preserved firmware. The driver sends UDP to `192.168.4.1:4210` and listens on local port 5005. Every message is a newline-terminated UTF-8 string. Check firewall and network routing if telemetry is absent.

| Input | Action/message |
|---|---|
| Axes 0/1 | Dominant-axis F/B/L/R, deadzone 0.25; S when centered; sent only when the command changes |
| A (index 0) | Toggle wall mode W / joystick J |
| B (index 1) | Toggle Vive V / joystick J |
| RB (index 5) | G: arm wall-follow motion in wall mode |
| LB (index 4) | T: toggle continuous servo sweep in joystick mode |
| Terminal in Vive mode | `x,y` → `TARGET:x,y`; `x1,y1 x2,y2` → `TARGET2:x1,y1,x2,y2` |
| Terminal in Vive mode | AL / AH / AN: low tower / high tower / nexus timed routine |

Target coordinates and the 100-unit arrival tolerance are in the Vive interface's coordinate units; the firmware does not establish a calibration from those units to metres. Two sensor positions define the center and heading. A two-target queue is implemented; a general graph planner is not connected to this final firmware. An earlier A* helper exists in the archive but is not part of the demonstrated integrated runtime.

## Wiring from actual source

| Master GPIO | Function |
|---|---|
| 3 / 7 | Left / right motor PWM, 0–255 command range |
| 5,6 / 36,35 | Left front / left back H-bridge direction pairs |
| 33,38 / 18,17 | Right front / right back direction pairs |
| 11 | Servo PWM: 50 Hz, 12-bit duty, 600–2400 µs pulses; 20 µs steps every 20 ms |
| 12 / 14 | Two Vive signal inputs |
| 44 RX / 43 TX | UART1 from sensor slave, 115200 baud, 8N1 |
| 8 SDA / 9 SCL | Optional TopHat I²C packet-count output to 0x28, 40 kHz |

| Slave GPIO | Function |
|---|---|
| 4 trigger / 5 echo | Right-back ultrasonic |
| 8 trigger / 10 echo | Right-front ultrasonic |
| 7 SDA / 6 SCL | Front VL53L0X I²C, 400 kHz |
| 20 RX / 21 TX | UART0, 115200 baud, 8N1; connect TX21 to master RX44 and common ground |

The report uses level shifting for the ultrasonic echo signal. Confirm supply/logic levels and physical wiring before connection. The block diagram in the original report labels some interfaces differently; the table above follows the committed firmware. UART sends `DIST:RB,RF,F\n`, integer centimetres, with zero representing an invalid reading. The slave schedules acquisition every 70 ms, but blocking sensor reads mean this is not a guaranteed measured 14 Hz rate.

## Control behavior and limitations

Wall following uses median-of-three distances, a 10 cm right-wall target, a +3 cm right-back offset, distance/angle corrections, deadbands, smoothing and bounded PWM. A stable front reading below 25 cm for three control ticks invokes a left turn. Those ticks can reuse the same sensor sample. Missing right ranges fall back to straight driving; this is not a certified obstacle-avoidance system.

Vive navigation uses median and exponential filtering, heading-error rotate/drive states, a 3° alignment threshold and a 100-unit arrival threshold. Position noise and supply sensitivity are discussed in the report. An optional final ToF alignment branch contains a bounded loop followed by a timed forward motion.

AL/AH/AN are blocking fixed-duration motor sequences. They do not use Vive feedback, obstacle replanning or bumper feedback to stop the routine. Although the report calls the attack timing nonblocking, the committed source uses `delay()`: UDP, sensing and Stop processing are delayed during those functions. Preserve that distinction when reproducing them.

`S` sets motor PWM to zero but does not disarm wall following or cancel an active Vive target, so those controllers can command motion again. Switch to J before S for an idle manual state. There is no enforced UDP-loss watchdog (`lastRxMs` is recorded but not used to stop the robot); manual commands latch between updates. Software stop is not an independent emergency stop.
