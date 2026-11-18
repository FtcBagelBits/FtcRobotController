# PollenBot TeleOp

The driver-controlled part of the match. On the Driver Hub pick **PollenBot** (TeleOp list).
After autonomous it is already selected for you: just press INIT, then START.

![Controller during the match](img/teleop-controller.svg)

## Driving and shooting

| Control | What it does |
| --- | --- |
| **Left stick** | Drive forward/back and slide (strafe) left/right |
| **Right stick** | Turn |
| **Cross (A), tap** | Lower the flower drainer and run the intake inward at full power. Tap again to raise it and return intake control to the triggers. |
| **D-pad Up / Down** | Nudge the drainer one step. Reaching the opposite preset switches intake mode. |
| **Right trigger** | Intake in (squeeze harder = faster) |
| **Left trigger** | Intake out (spits pollen back out) |
| **R1, hold** | Shoot. The launcher spins up, then feeds by itself and keeps feeding until you let go. |
| **Square, hold** | **Unjam with drainer up:** runs the launcher slowly backwards to push a stuck ball out. While you hold Square nothing feeds, even if R1 is held too. |
| **Circle** / **Triangle** | Practice only: mark the **start** / **end** of a move we want to copy into autonomous. The gamepad buzzes once / twice. They do not change how the robot drives. |

**Drain a flower:** Lower the drainer, guide it into the flower, then rock the left stick gently left and right. Tap Cross to raise the drainer when finished.

**Shooting:** hold R1 the whole time you want to shoot. Feeding starts as soon as the launcher
is up to speed, or after 3 seconds at most. Let go of R1 to stop.

**Jammed?** Let go of R1 and raise the drainer. Hold Square for a second or two. You can also squeeze the left trigger to push
balls out of the intake. Let go of Square, then hold R1 to shoot again.

## The INIT menu (before START)

You can change these settings before START. Usually leave them alone.

| Row | Default | What it changes |
| --- | --- | --- |
| Launcher ticks/s | 1180 | How fast the launcher spins (changes by 20) |
| Spin-up before feed | 3.0 s | Longest wait before feeding starts |
| Agitator power while feeding | 1.00 | How hard the agitator pushes balls in |
| Drive power | 100% | Lower it for slower, more careful driving |
| Drainer up / down / step | 0.45 / 0.02 / 0.01 | Raised position, lowered position, and distance per D-pad tap |

- **D-pad UP/DOWN:** pick a row (marked `>`). **D-pad LEFT/RIGHT:** change it. Tap and let go.
- **Triangle:** all back to the defaults. The screen says `Settings: DEFAULTS` or `CHANGED - check before START`.
- Settings stay until the robot app restarts, so **check the INIT screen before every match**.
- The screen should show **Build `pollen-hive-v20`**.
- **Keep the robot still during INIT.** The robot records every TeleOp run for the programmers.
  If the screen shows `STORAGE WARNING`, tell a mentor after the match; the robot still drives.

## What the screen tells you during the match

| Feed line says | Meaning |
| --- | --- |
| `off (hold R1; Square = reverse to clear a jam)` | Not shooting |
| `spinning up` | R1 held, launcher getting up to speed |
| `FEEDING (started at 1.2 s: up to speed)` | Shooting |
| `FEEDING (... spin-up timer)` every time | Launcher never gets fully up to speed. Tell a programmer: check the battery and launcher cable. |
| `Raise drainer to unjam (Square)` | Drainer is down. Raise it to enable launcher reverse. |
| `REVERSING launcher (Square) - no feed` | Unjamming |

`(measured ~0: check launcher encoder cable)` next to the launcher speed means the launcher's
encoder cable is loose. Shots will be weak or uneven: tell a programmer after the match.
