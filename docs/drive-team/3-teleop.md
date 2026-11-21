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
| **Right trigger** | Intake in (squeeze harder = faster). Manual agitator power tops out at 0.50. |
| **Left trigger** | Intake out (spits pollen back out) |
| **R1, hold** | Shoot. The launcher spins up, then feeds by itself and keeps feeding until you let go. |
| **L1, hold** | **Unjam, drainer up first** (Square raises it): runs the launcher slowly backwards to push a stuck ball out. While you hold L1 nothing feeds, even if R1 is held too. |
| **Circle, tap** | **Aim at the hive. Relatively untested: use with care, and don't count on it in a match.** Turns in place until the launcher points at the nearest hive cell the Limelight sees. One buzz = aiming, one blip = done, three blips = no hive tags in view (nothing happens). The left stick still drives; the right stick cancels it. Aims 2 in right of the cell's middle by default (INIT row **Circle aim target in left of center**). |
| **Square, tap** | **Drainer up** and intake back to the triggers. Cross still toggles up/down. |
| **Triangle** | Does nothing during the match. |

**Drain a flower:** Lower the drainer, guide it into the flower, then rock the left stick gently left and right. Tap Cross to raise the drainer when finished.

**Shooting:** hold R1 the whole time you want to shoot. Feeding starts as soon as the launcher
is up to speed, or after 3 seconds at most. Let go of R1 to stop.

**Jammed?** Let go of R1 and tap Square (drainer up). Hold L1 for a second or two. You can also squeeze the left trigger to push
balls out of the intake. Let go of L1, then hold R1 to shoot again.

## The INIT menu (before START)

You can change these settings before START. Usually leave them alone.

In screen order. The first three are the ones to check (and maybe change); leave the rest alone.

| Row | Default | What it changes |
| --- | --- | --- |
| Launcher ticks/s | 1180 | How fast the launcher spins (changes by 20) |
| Drive power | 100% | Lower it for slower, more careful driving. **Check it is 100%.** |
| Circle aim target in left of center | -2.0 (2 in right) | Where Circle aims, in 0.5 in steps. Negative = right. Change if Circle-aimed shots land to one side. |
| Spin-up before feed | 3.0 s | Longest wait before feeding starts. Leave alone. |
| Agitator power while feeding | 1.00 | How hard the agitator pushes balls in. Leave alone. |
| Drainer up / down / step | 0.45 / 0.02 / 0.01 | Raised position, lowered position, and distance per D-pad tap. Leave alone. |

- **D-pad UP/DOWN:** pick a row (marked `>`). **D-pad LEFT/RIGHT:** change it. Tap and let go.
- **Triangle:** all back to the defaults. The screen says `Settings: DEFAULTS` or `CHANGED - check before START`.
- Settings stay until the robot app restarts, so **check the INIT screen before every match**.
- The screen should show **Build `pollen-hive-v44`**.
- **Keep the robot still during INIT.** The robot records every TeleOp run for the programmers.
  If the screen shows `STORAGE WARNING`, tell a mentor after the match; the robot still drives.

## What the screen tells you during the match

| Feed line says | Meaning |
| --- | --- |
| `off (hold R1; L1 = reverse to clear a jam)` | Not shooting |
| `spinning up` | R1 held, launcher getting up to speed |
| `FEEDING (started at 1.2 s: up to speed)` | Shooting |
| `FEEDING (... spin-up timer)` every time | Launcher never gets fully up to speed. Tell a programmer: check the battery and launcher cable. |
| `Raise drainer (Square) to unjam with L1` | Drainer is down. Tap Square, then hold L1. |
| `REVERSING launcher (L1) - no feed` | Unjamming |

`(measured ~0: check launcher encoder cable)` next to the launcher speed means the launcher's
encoder cable is loose. Shots will be weak or uneven: tell a programmer after the match.
