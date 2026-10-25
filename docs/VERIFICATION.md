# Executed verification

October 24, 2026. No robot was connected or commanded. Historical hardware footage and report observations are kept separate from these software/build checks.

| Check | Result |
|---|---|
| Python 3.12.14 / Pygame 2.6.1 | Three unittest methods pass: direction/deadzone behavior (ten subcases), newline UDP formatting/destination using a fake socket, and original-source SHA-256 integrity |
| Original sources/report | Controller_Python.py, both .ino files and MEAM_5100_final_report.pdf match docs/source_manifest.json (LF-normalized text, exact binary report) |
| Master firmware | Arduino-ESP32 3.3.2, esp32:esp32:um_feathers2: compiled; 933,054 flash bytes / 62,100 RAM bytes |
| Slave firmware | Arduino-ESP32 3.3.2, esp32:esp32:m5stack_stamp_c3, Adafruit_VL53L0X 1.2.5: compiled; 326,654 flash bytes / 13,976 RAM bytes |
| Media | All supplied archive figures and eight timestamps across each of three complete clips inspected; display media uses actual photos/frames |

Builds used Arduino CLI 1.5.2-rc.1 downloaded from the official Arduino endpoint. The master uses the supplied Vive interface located in final project/vive1, staged locally with its credits unchanged. No source, board or control-law substitutions were made.

The initial slave build failed in the RISC-V toolchain at `bits/c++config.h`. The file existed at a long installed path. Retrying with a short local junction to the same installed toolchain and a compiler.path override resolved the failure. This is an environment workaround, not a firmware repair. On this Windows checkout:

```powershell
# TOOLCHAIN_ROOT is the installed esp-rv32/2507 directory; TOOLCHAIN_ALIAS is a short local path.
New-Item -ItemType Junction -Path TOOLCHAIN_ALIAS -Target TOOLCHAIN_ROOT
arduino-cli compile --fqbn esp32:esp32:m5stack_stamp_c3 --build-property 'compiler.path=TOOLCHAIN_ALIAS/bin/' 'outputs/firmware/slave'
```

Local detailed compiler logs and build artifacts stay under ignored outputs/. Machine-readable summaries are in verification.json. These builds verify compatibility with the named targets and dependencies; they do not verify electrical wiring, board identity on a connected robot, timing under load, sensing accuracy or hardware performance.

## Source/report reconciliation

The supplied archive includes earlier standalone experiments and an A* waypoint helper. Its Final_Code sketches match the repository after newline normalization; byte differences are line endings. These are not imported as if they formed a tested integrated planner. Final repository firmware remains the executed build baseline. The report's statement that attack routines are nonblocking conflicts with the committed delay-based functions; the operating guide documents the actual behavior. Three control ticks for front confirmation are not three independent sensor acquisitions. Stop handling and absent link-loss watchdog behavior are also documented from code.

Developed as a Penn MEAM 5100 team project. Existing original source/report credits remain intact.
