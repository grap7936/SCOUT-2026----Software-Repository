# SCOUT INSTR

Software and operating instructions for the SCOUT INSTR debris-tracking system. The system runs on a Jetson (Orin) Nano paired with an Arduino Nano and an Alvium camera, detecting and tracking debris objects and driving a gimbal motor to follow them.

## Table of Contents

- [System Setup](#system-setup)
  - [Supplying Power](#supplying-power)
  - [Connecting to the Jetson](#connecting-to-the-jetson)
- [Using the Software](#using-the-software)
  - [Testing](#testing)
  - [Primary Program](#primary-program)
- [Post-Processing Data](#post-processing-data)
  - [Saving the Data](#saving-the-data)
  - [MATLAB Script](#matlab-script)
  - [Log File Formats](#log-file-formats)
- [Modifying the Software](#modifying-the-software)
  - [Arduino](#arduino)
  - [Jetson](#jetson)

## System Setup

### Supplying Power

The SCOUT INSTR system is designed to accept 12V DC power in with a 2A max current draw to its primary power connector.

All subsequent components have power distributed from the PDN board or the Jetson Nano.

Ensure that the camera and Arduino Nano are connected via USB to the Jetson Nano. Ensure the camera JST cable is connected to the Arduino (pin D4 as of this writing).

Ensure the Jetson and Arduino Nano both show LEDs indicating power is supplied.

### Connecting to the Jetson

Allow 1–2 minutes for the Jetson Nano to boot up and start its WiFi hotspot.

Look in your WiFi connections menu for a network named `Scout_INSTR` and connect. The password is also `Scout_INSTR`.

Once connected, you can either connect via terminal (PowerShell) or VNC (RealVNC Viewer).

If using RealVNC Viewer, connect to the following IP address:

```
10.42.0.1:5900
```

If using a terminal, enter the following command:

```
ssh scout@10.42.0.1
```

In both cases, when prompted for a password, use `scout`.

Now you've successfully logged in.

### Software Configuration

The `Makefile` has build dependencies the user must consider. Anyone recompiling on a fresh Jetson will need these installed:

- **g++** with C++17 support and OpenMP (`-fopenmp`), compiled with `-O2 -march=native`.
- **OpenCV 4** (headers expected at `/usr/local/include/opencv4`, resolved via `pkg-config opencv4`).
- **Vimba X SDK** for the Alvium camera, expected at `/home/scout/vimba-x/VimbaX_2026-2/api/` (links `-lVmbCPP -lVmbC`). The runtime `rpath` is baked to that path, so the SDK must remain in that location.

If the Vimba X path or version changes, the include/library paths in the `Makefile` must be updated to match.

`main.cpp` expects the Arduino on serial port `/dev/ttyCH341USB0`. A code comment notes that on some boards the Arduino may instead enumerate as `/dev/ttyACM1` (or similar). If the system fails to connect to the Arduino, verify the actual serial device (e.g. with `ls /dev/tty*`) and update the `serial_port` value in `main.cpp` accordingly.

`main.cpp` sets a user-defined capture rate of `FPS = 50.0`. The Alvium camera's maximum is noted as 79.0 FPS. Adjust this constant in `main.cpp` if a different frame rate is needed (requires recompile).

See [Modifying the Software](#modifying-the-software) for how to make code edits.

## Using the Software

To run the software, navigate to the INSTR directory. Enter the following in the terminal:

```
cd ~/Desktop/INSTR
```

Your terminal session should now be in the INSTR directory.

Now you can run the driver script that manages calling the relevant programs:

```
./driver.sh
```

You should see the script run and prompt you with a menu of options to select. These are detailed in the following sections.

### Testing

The `driver.sh` script's Standby menu currently exposes these options in order: **TestArduino, TestCam, IdleCam, DebrisTracking, TestAlgorithm, Power Off**. Note that `TestAlgorithm` runs a sequence of lower-level component tests (Detector → Graph → Selector → Full test) rather than a single algorithm run. The full state tree is documented in comments at the top of `driver.sh`.

**TestArduino** — Custom program to test Arduino motor control and comms functionality. This program will prompt the user with another menu of different tests:

- `-1`: spin the motor at constant speed
- `-2`: do one revolution from 0 to 2·pi (no PID)
- `-3`: send a ping of data packets over the serial connection
- `-4`: test PID with a few set positions
- `-5`: exit testing command lock — all positive numbers sent will be acted upon
- `-6`: re-enter testing command lock — positive numbers ignored

**TestCam** — Tests basic camera functionality. Takes a single image saved as `testCamImage.png` in the `INSTR/` folder.

**IdleCam** — Tests camera functionality with the algorithm. Runs the algorithm on a live camera capture without driving the motor.

**TestAlgorithm** — Tests the algorithm on sample input. Runs the algorithm and propagates target log files (`oldTargetsLog.txt`, `debrisLog.txt`), saving a modified video with debris scores to `debrisTestResults.ts`.

### Primary Program

**DebrisTracking** — Runs the full algorithm using all peripheral hardware (camera, Arduino, motor). While running, this program can be terminated anytime by typing `q` into the terminal. This program propagates all log files (`oldTargetsLog.txt`, `debrisLog.txt`, `motorLog.txt`) and saves video to `debrisTestResults.ts`.

## Post-Processing Data

### Saving the Data

To copy data files from the Jetson Nano to your local computer, open a PowerShell terminal. If you still have an active SSH session from earlier setup, first log out:

```
logout
```

To copy the relevant files (`debrisLog.txt`, `motorLog.txt`, `oldTargetsLog.txt`, `debrisTestResults.ts`), use the secure copy command:

```
scp -r scout@10.42.0.1:~/Desktop/INSTR/*.t* C:\Users\<Your_user>\Desktop\
```

Remember to replace `<Your_user>` with your local Windows machine username or you will see an invalid file path error.

### MATLAB Script

Once the `debrisLog` and `motorLog` files are downloaded, pass them into the `trajectoryCalc.m` script, which will find the estimated trajectories of all debris objects (if possible).

### Log File Formats

For anyone writing a custom post-processing script, the log files written by the primary program use these column headers:

- `oldTargetsLog.txt`: `id, x, y, kx, ky, vx, vy, score`
- `debrisLog.txt`: `frame_num, id, x, y, kx, ky, vx, vy, score`
- `motorLog.txt`: `frame_num, motor_pos, gap`


## Modifying the Software

Whenever you would like to add, modify, or remove code, there is a straightforward process to recompile the software afterwards.

### Arduino

For the Arduino, this is handled in the Arduino IDE. Simply compile all Arduino files (`ArduinoMain.ino`, `ReceiveEnd_Arduino.hpp`, `ReceiveEnd_Arduino.cpp`) onto the Nano.

**`arduino/`** — stores a copy of the current Arduino code.

### Jetson

For the Jetson, the process is a bit more involved. For new files, ensure your software is distributed into the file structure as follows:

- **`src/`** — where the core algorithm code lives; all non-testing `.cpp` files live here.
- **`include/`** — where all header files corresponding to `.cpp` files found in `src/` go. There can also be additional header files here that don't have a corresponding `.cpp`.
- **`testing/`** — where algorithm-testing-related `.cpp` files go.
- **`bin/`** — where program executables are compiled to.
- **`INSTR/`** — houses other program files like `driver.sh`, `Makefile`, and more. Hardware testing `.cpp` files should be placed here (`testingArduinoMain.cpp`, `IdleCam.cpp`, etc.).

After adding your code, recompile.

If you are looking to change algorithm parameters, these live in the `Sentry.cpp` file in the class constructor declaration. All parameter values are hard-coded here, but can also be set with helper functions. If changing the hard-coded parameters, you will need to recompile.

To recompile code on the Jetson, log in via SSH or VNC (see [System Setup](#system-setup)). Once in the Jetson terminal, navigate back to `~/Desktop/INSTR` and run:

```
make clean && make
```

This will take a few minutes to compile.
