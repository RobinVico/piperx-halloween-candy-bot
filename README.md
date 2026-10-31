# 🎃 PiPER X Halloween Candy Bot (+ SO-101 teleoperation)

Windows-only toolkit for driving an **AgileX PiPER X** arm from an open-source **SO-101 leader arm**,
plus a **Halloween candy bot**: when someone holds out an open palm, the arm picks a candy from a box
and drops it into their hand.

*中文：用开源 SO-101 主臂遥操作 AgileX PiPER X 机械臂（Windows），以及万圣节递糖机器人——摄像头看到摊开的手掌，机械臂就从糖盒里夹一颗糖放到手里。详细中文操作手册见 [RUNBOOK.md](RUNBOOK.md)。*

## Features
- **SO-101 → PiPER X teleoperation** with a smooth, speed/acceleration-limited safety filter
  (joint limits, glitch rejection, leader-loss hold, tracking-error auto e-stop).
- **Always-on-top emergency control panel** (Tk): big e-stop (ESC), pause, home-and-disable,
  speed slider, live status, base (J1) zero calibration, waypoint teaching.
- **MuJoCo simulation** of the PiPER X built from the official URDF: sim teleop, dry runs, and a
  "mirror" view of the real arm.
- **Candy bot**: MediaPipe hand detection on a RealSense D415 triggers a pick → hand-off →
  release state machine over taught waypoints; releases only when a hand is under the gripper,
  keeps the candy for the next person otherwise; restartable sessions.
- **Orbbec Dabai DC1** colour + depth preview (gripper camera).

## Hardware
| Part | Notes |
|---|---|
| AgileX PiPER X (firmware S-V1.8-9) | spherical wrist: **J4 = pitch, J5 = yaw, J6 = roll** |
| candleLight USB-CAN (1D50:606F, WinUSB) | used via the `candle` python-can interface, 1 Mbit/s |
| SO-101 leader arm (Feetech STS3215) | lerobot calibration, COM port |
| Intel RealSense D415 | fixed camera watching the hand-off zone (USB 3) |
| Orbbec Dabai DC1 | on the gripper; depth needs Orbbec SensorDriver + OpenNI2 SDK |

## Quick start (Windows, PowerShell)
```powershell
py -3.12 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
```
Then use the launchers:

| Launcher | What it does | Real arm moves? |
|---|---|---|
| `1_check_connections.bat` | CAN / PiPER / SO-101 checks | no |
| `2_sim_teleop.bat` | SO-101 drives the simulated arm | no |
| `3_dry_run_panel.bat` | panel + sim, real arm connected but never enabled | no |
| `4_LIVE_teleop_panel.bat` | real teleop (enable only after confirm in panel) | **yes** |
| `5_mirror_real_arm.bat` | real arm pose shown in sim | no |
| `6_candy_bot_LIVE.bat` | candy bot on the real arm | **yes** |
| `7_candy_bot_dry_run.bat` | candy bot with the simulated arm | no |

Teach the candy-bot waypoints (`home`, `box_above`, `grasp_1..3`, `handoff_above`) with the
"记录点位" button in the teleop panel. See [RUNBOOK.md](RUNBOOK.md) for the full procedure,
troubleshooting and Windows-specific gotchas.

## Layout
```
teleop/   mapping + safety filter, PiPER X driver, SO-101 reader, teleop panel
sim/      MuJoCo model builder, viewer, sim teleop, real-arm mirror
candy/    candy bot state machine + panel, taught waypoints
vision/   RealSense/UVC cameras, MediaPipe palm detection + trigger
tools/    connection checks, camera / RGB-D / palm previews
```

## ⚠️ Safety
This drives a real robot arm near people (children, for the candy bot). Keep the speed low, keep an
adult at the e-stop, release candy from above the hand (never touch it), and never call
`ResetPiper()` / `EmergencyStop(0x02)` — the arm loses power and drops. Use at your own risk.

## Credits & licenses
Own code: [MIT](LICENSE). Third-party assets: see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)
(PiPER X meshes from [agilexrobotics/agx_arm_urdf](https://github.com/agilexrobotics/agx_arm_urdf), MIT;
MediaPipe hand landmarker model, Apache-2.0).
Built with [LeRobot](https://github.com/huggingface/lerobot), [piper_sdk](https://github.com/agilexrobotics/piper_sdk),
[MuJoCo](https://github.com/google-deepmind/mujoco), [MediaPipe](https://github.com/google-ai-edge/mediapipe).
