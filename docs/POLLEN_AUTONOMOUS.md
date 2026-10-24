# Pollen autonomous operating manual

All three autonomous modes and PollenBot TeleOp show build **`pollen-mat-v3`**.
Keep these instructions with the installed code. The new mat calibration and field routes are **experimental**.
The previously tested B settings remain its default and are restored by Triangle.

## Choose the routine

| Driver Hub choice | Behavior | Starting pose |
| --- | --- | --- |
| **Pollen ROBOT Park** (A) | Selected delay, timed forward drive, hold | Manually aligned with the team's tested forward parking path |
| **Pollen ROBOT Shoot + Park** (B) | Delay, stationary shot, up to three selected drive legs, hold | End wall, flush and square, centerline aimed into the initially upward-facing cell |
| **Pollen Supporting Shooter Auto** (C) | Wait with flywheel prespin, stationary forward shot, forward flower clearance, mostly-right travel, hold | Opposite end wall, **flush and square, centered between flower and midline** (below); partner tips the cell before our shot |

All preselect **PollenBot** for TeleOp, without automatically starting it. There
is no localization, heading correction, obstacle detection or hive-tip sensing.
The same robot-relative controls work for blue when the whole red pose is
rotated 180 degrees. Humans must establish the correct pose before START.

**Full-field placement is in [POLLEN_FIELD_ROUTES.md](POLLEN_FIELD_ROUTES.md), with
an [overview diagram](POLLEN_FIELD_ROUTES.svg).** A direct primary diagonal
intersects the frame. A centered supporting start overlaps a flower. Do not
substitute either of those poses/routes for the field presets.

## Before every run

1. Connect the charged robot and Driver Hub; keep the working Pollen hardware
   configuration. Enable the Driver Station autonomous timer.
2. Select the mode, place the robot for that mode and press INIT. Check the
   build, hardware status, remembered settings, calibration profile and time budget.
3. Tap/release controller buttons to edit. Motors remain stopped during INIT.
   Photograph the settings; they lock at START.
4. Confirm the entire footprint, shooting area and partner route are clear.
   For C, physically verify the tight flower/midline clearance of the flush
   pose (about 2.6 cm each side nominally). The CAD clearance does not establish clearance on the actual field.
5. Agree lanes with the partner using the
   [pit sheet](POLLEN_PIT_SHEET.svg): who shoots first, inner/outer parking
   slot, and that our C feeds from t=15 s. A robot touching the side wall at
   the end of AUTO loses its 3 LEAVE points (and the alliance its 16-point SWARM).
6. START runs the selected sequence. STOP interrupts it. At DONE, commands stay
   zero; DONE does not prove parking or scoring. Press STOP before another trial.

## A controls and defaults

| Button | Action |
| --- | --- |
| D-pad UP / DOWN | Delay +/-0.5 s |
| D-pad RIGHT / LEFT | Forward time +/-0.05 s |
| R1 / L1 | Power +/-0.05 |
| Triangle | Restore A defaults |

Defaults remain **2.00 s, power 0.20, delay 0**. Time range is 0.05-5.00 s;
power range 0.05-0.50. Delay is limited to 20 s and the route budget. A holds
zero outputs for 0.5 s after driving.

A still uses time, not requested distance. Its new **Mat forward estimate** uses
the measured 0.25-power fit (138.17 cm/s per unit power), interpolating toward
the one 0.50-power point (150.67). Below 0.25 it holds the lower coefficient and
warns that this is extrapolation. The default display estimates **55.27 cm**;
the motor commands are unchanged. See [calibration analysis](POLLEN_MAT_CALIBRATION.md).

## B and C controller controls

- **D-pad UP / DOWN:** select the row marked `>`.
- **D-pad RIGHT / LEFT:** edit the selected row. Tap/release; held keys do not repeat.
- **Triangle:** restore that mode's defaults. B restores its tested short baseline;
  C restores its experimental supporting field preset.
- **Square:** B loads the experimental three-leg field preset, replacing all
  selections. C: same as Triangle. It does not move the robot. Check every row.

Every speed, distance and angle that the routes use is a row, so tuning needs
no rebuild. After the shot the robot drives **leg 1, leg 2, leg 3** in order,
all at the one drive power; a leg set to 0 cm is skipped.

| Row | B default (Triangle) | B Square preset | C default | Increment / range |
| --- | ---: | ---: | ---: | --- |
| Launcher ticks/s | 1250 | 1250 | 1250 | 50; 500-2500 |
| Feed seconds | 10 | 10 | 7.5 | 0.5; 0-20; 0 skips shooting |
| Intake + agitator power | 0.50 | 0.50 | 0.50 | 0.05; 0.10-1.00 |
| Start delay seconds | 0 | 0 | 15 | 0.5; 0-20, subject to budget |
| Calibration | 0: tested baseline | 1: MAT_TRIAL | 1: MAT_TRIAL | 0 or 1 |
| Drive power, all legs | 0.20 | 0.25 | 0.45 | 0.05; 0.10-0.50 |
| Leg 1 cm, off wall | 0 | 10 | 20.32 | 1; 0-60 |
| Leg 1 angle deg (+left) | 0 | 0 | 0 | 1; -90 to +90 |
| Leg 2 cm, side clear | 0 | 63.50 | 0 | 5; 0-150 |
| Leg 2 angle deg (+left) | +90 | +90 | -90 | 1; -90 to +90 |
| Leg 3 cm, to park | 60.96 | 217.47 | 117.96 | 5; 0-350 |
| Leg 3 angle deg (+left) | +25.87 | +12.313 | -81.848 | 1; -90 to +90 |

Fractional angles step to the next whole degree first, then by 1, so exactly 0
is reachable. To shoot in place, set **all three leg distances to zero**. To test
driving without shooting, set Feed seconds to zero. Telemetry shows each leg as
`cm @ angle -> seconds; command angle`, or `skipped (0 cm)`.

Angles describe translation relative to the starting chassis heading, not a
chassis turn. Positive means robot-left; negative robot-right. MAT_TRIAL adjusts
the motor-command angle to estimate the desired physical direction and changes
duration by direction. Baseline preserves the old `cm / (129 * power)` timing
and direct sine/cosine command. MAT_TRIAL assumes unconfirmed original test
settings: verify it with short trials before relying on a field route.

## Primary field preset: Square in B

The default 60.96-cm baseline is a short test, not a full-field park. Square loads:

- MAT_TRIAL, power **0.25**, normal 10-second feed and zero delay;
- leg 1: **10 cm forward** off the wall after shooting;
- leg 2: **63.50 cm left** to clear the frame;
- leg 3: **217.47 cm at +12.313 degrees** to the near parking slot.

This avoids the floor-level frame bar in the nominal CAD. It budgets about
**24.20 s** including worst-case spinup and hold. Triangle restores the tested
short route (leg 3 only). Editing any leg changes the route; check the geometry
again after such edits.

## Supporting field preset: C

Start **flush against the scoring end wall and square to it**. Slide the robot
toward the midline from the hive center until the gap to the flower equals the
gap to the alliance midline: nominally **2.6 cm each side**, which puts the robot
centerline **6.9 cm toward the midline** from the hive center. The shot goes
straight forward and lands about 6.9 cm off the cell center (opening about 51 cm
wide) at the same distance as a centered start. Measure on the real field:
the total slack between flower and midline is only about 5.2 cm.

Default sequence, ignoring small loop-transition delays:

| Time from START | Action |
| --- | --- |
| 0-11 s | All motor commands zero |
| 11-15 s | Flywheel spins; drive, intake and agitator remain off |
| 15-22.5 s | Feed for 7.5 s if speed qualified; remain stationary |
| After 22.5 s | Stop all shooting mechanisms; leg 1: 20.32 cm forward to clear flower |
| Next | Leg 2 skipped (0 cm); leg 3: 117.96 cm at -81.848 degrees (mostly right) |
| End | Hold zero outputs for 0.5 s, then DONE |

This uses power 0.45 and MAT_TRIAL; predicted total is about **25.85 s**, so C
has almost no budget left: longer legs or slower power need a shorter delay or
feed. Neither this power/direction combination nor the route has been validated
on the robot. The partner's hive tip is an assumption, not a detected event.

If the launcher is not ready at the end of the delay, C skips the shot and
waits through the selected feed slot before departing. Shooter INIT fallback
also retains that slot. If you deliberately set Feed seconds to zero, both
prespin and the feed slot disappear: departure starts after the selected delay.
With feeding enabled, delay cannot be reduced below four seconds.

## PollenBot TeleOp INIT settings

PollenBot TeleOp has a two-row INIT menu, set before START like the autonomous
menus. Driving controls are unchanged; the menu only changes two numbers.

| Row | Default | Increment / range | Effect during the match |
| --- | ---: | --- | --- |
| Launcher ticks/s | 1250 | 50; 500-2500 | Target while R1 is held. Feeding boost starts at 96% of it (1200 at default). |
| Drive power | 100% | 5%; 20-100% | Multiplies all wheel powers after the usual mixing, so slower but same stick feel. |

- **D-pad UP / DOWN:** select the row marked `>`. **RIGHT / LEFT:** edit. Tap/release.
- **Triangle:** restore both defaults. INIT shows `DEFAULTS` or `CHANGED`.
- Settings lock at START and show on the match screen with the measured launcher
  speed. They are kept for the next INIT until the Robot Controller app restarts,
  so check the INIT screen before every match.

A faster launcher target only helps if the motor can reach it under load: watch
"Launcher target / measured". If measured stays below 96% of target, feeding
never gets the boost. Find the best speed on the practice field and record it.

## Shooting, memory and failure handling

The launcher qualifies at 96% of target for 0.25 seconds. Then launcher, main
intake and agitator run for the full feed duration; ordinary speed dips do not
pause feeding. No windmill feeder is used. Encoder target is ticks/s, not RPM;
feed time is commanded motor time, not a ball count.

B spins up after its delay, with a four-second timeout. Timeout skips the shot
and continues its selected route. C spends that spinup allowance inside the
last four seconds of its delay. Invalid velocity data stops either mode.

A, B, C and PollenBot TeleOp remember settings separately while the Robot
Controller app runs.
Start delay resets every INIT: zero for A/B, 15 for C. If C's remembered route
would exceed the budget with restored delay, all C defaults are restored with
a visible notice. App restart/reinstall loses memory. Triangle is not a recall
of the last successful match settings.

Edits are rejected if worst-case total reaches **26 seconds**. All three legs
count toward this budget. An independent **27-second cutoff**
stops outputs. This is the program's guard, not the official autonomous duration.

| Warning | Meaning |
| --- | --- |
| BLOCKED | Drive/hardware initialization failed, or a shooter stop could not be confirmed. |
| SHOOTER FAILED / DEPARTURE ONLY | Shooting setup failed but all mapped shooter motors accepted stop commands. The mode can run its own selected route; C also waits through its feed slot. |
| Shot skipped / retaining slot | C did not qualify speed; it stays still until the planned feed slot ends. |
| Change rejected | Requested settings exceed budget or violate C's prespin requirement. |
| Hardware error | Attempts to zero every device and requests OpMode stop. |

## Verification and settings record

Test INIT and STOP, then short isolated drive/shot trials, then each field leg,
then the combined route. The parked robot must end partly on the tape and off
the side wall: the nominal margins are about 14 cm each way, roughly 12% of the
lateral travel, so tune leg distances until repeated runs stay well inside. Confirm zero shooter output before driving. Repeat
with actual load, expected battery and the partner's route. Verify both alliance
placements. Do not equate computer tests or SDK compilation with a tested route.

Record: installed commit/build; date; battery; surface/load; A/B/C; measured start
center/heading; profile; launcher/feed settings; delay; drive power; each leg's cm/angle; actual forward/side displacement; final heading; endpoint;
shots and STOP result; partner parking slot; who verified the complete route.
For TeleOp, record the launcher ticks/s and drive power that worked best.

Build/deployment instructions are in the
[package README](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/README.md).
