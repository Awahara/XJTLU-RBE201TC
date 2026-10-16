# XJTLU RBE201TC — RDK X5 Robot Control

Coursework repository for the **RBE201TC** module: motor, line-tracking and
environmental-sensor control for a four-wheeled robot built on a
**D-Robotics RDK X5**.

> **Status:** this repo currently contains the motor-control tool and the
> engineering log written during bring-up. The laboratory code package
> (`line_follower.py`, `line_sensor_test.py`, `moto_test.py`, the two DHT11 C
> programs) is **not included yet** — it lives on the robot and will be added in
> a follow-up commit.

---

## ⚠️ Read this before powering the motors

**Never send `$upload:1,0,0#`.** That single command permanently breaks the
motor driver board's receive channel:

* encoder telemetry starts streaming at ~150 Hz, and
* every telemetry frame is answered with `fail,Command not found!`, and
* from then on **no command is accepted — including the stop command.**

This is not theoretical. It is exactly how a motor was left running with no
software way to stop it during this project. `$upload:0,0,0#` does **not** undo
it; only a full power cycle (unplug the Type-C **and** remove battery power)
restores the board.

**Second:** a **single** `$spd:0,0,0,0#` does **not** reliably stop a motor.
A stop must be sent as a **burst**. See `docs/motor-control-notes.md`.

**Third:** there is no software emergency stop. The only guaranteed stop is
cutting power to the driver board. Keep the switch within reach.

---

## Hardware

| Item | Detail |
| --- | --- |
| Main board | D-Robotics RDK X5 V1.0 — Ubuntu 22.04.5, kernel 6.1.83, 8-core aarch64 |
| Motor driver | Yahboom 4-channel encoder motor driver, **onboard CH340K** (`1a86:7522`) |
| Motor link | USB Type-C → `/dev/ttyUSB0`, 115200 8N1, no flow control |
| Line tracking | 4 reflective IR sensors, `GPIO.BOARD` pins **15 / 18 / 16 / 11** (black tape reads `0`) |
| Environment | DHT11 temperature/humidity sensor on Linux GPIO **401** |

The driver board's MCU is powered from the USB Type-C connection, so unplugging
the motor battery does **not** reset it.

## Motor control protocol

ASCII commands framed as `$` + payload + `#`, at 115200 baud.

| Command | Effect | Reply |
| --- | --- | --- |
| `$mtype:1#` | set motor type | `Set mtype:OK!` |
| `$spd:m1,m2,m3,m4#` | set the four speeds, range −1000…1000 | **none** (by design) |
| `$pwm:m1,m2,m3,m4#` | alternative drive command | **none** |
| `$upload:1,0,0#` | ⛔ **do not use** — breaks the receive channel | telemetry stream |
| `$stop#` | ⛔ not supported by this firmware | `fail,Command not found!` |

Because `$spd` never acknowledges, **its effect cannot be confirmed in
software.** Enabling telemetry to get feedback is what breaks the board, so
there is no usable closed loop: the operator must watch the robot.

Note that `Rosmaster_Lib` and `SunriseRobotLib` (both installed on the board)
speak a *binary* `FF FC …` protocol which this board does not understand.

## Contents

```
.
├── README.md
├── motor_ctl.py                     # motor control tool (safe by construction)
└── docs/
    └── motor-control-notes.md       # full engineering log (Chinese)
```

`docs/motor-control-notes.md` records the whole investigation: the failure
timeline, the evidence that identified `$upload:1,0,0#` as the root cause, the
measured command-reliability behaviour, a post-mortem of the wrong conclusions
reached along the way, and the resulting safety procedure. It is the most useful
document here.

## Usage

Copy `motor_ctl.py` to the robot and run it there:

```bash
# read-only: listens, sends NOTHING
python3 motor_ctl.py check

# liveness only: sends $mtype:1#, which cannot drive a motor
python3 motor_ctl.py probe

# drive one channel: m1 m2 m3 m4 seconds  (everything is sent as a burst)
python3 motor_ctl.py raw 250 0 0 0 2

# stop: sends a burst of 80 x $spd zeros + 40 x $pwm zeros
python3 motor_ctl.py stop
```

Design rules the tool enforces:

* every speed command is repeated (5×) — single commands proved unreliable;
* every stop is a heavy burst — the only stop form observed to work;
* it **never** sends `$upload:1,0,0#`;
* it never claims a motor stopped; there is no software confirmation available.

## Verification so far

All four motor channels were individually driven and confirmed to turn
(M1, M2, M3, M4). Line-following code was reviewed and confirmed not to send the
dangerous command. A voice-control module is *not* present on this robot: there
is no second serial device and no related code on the system.
