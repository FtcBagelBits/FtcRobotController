# Pollen autonomous operating manual

All three modes show build **`pollen-mat-v1`**. Keep these instructions with the
installed code. The new mat calibration and field routes are **experimental**.
The previously tested B settings remain its default and are restored by Triangle.

## Choose the routine

| Driver Hub choice | Behavior | Starting pose |
| --- | --- | --- |
| **Pollen ROBOT Park** (A) | Selected delay, timed forward drive, hold | Manually aligned with the team's tested forward parking path |
| **Pollen ROBOT Shoot + Park** (B) | Delay, stationary shot, optional left clearance, selected angled drive, hold | End wall, square to wall, centerline aimed into the initially upward-facing cell |
| **Pollen Supporting Shooter Auto** (C) | Wait with flywheel prespin, stationary shot, forward flower clearance, mostly-right travel, hold | Opposite end wall, using the **offset-and-aim pose** below; partner tips the cell before our shot |

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
   For C, physically verify the tight flower/centerline clearance of the offset
   pose. The CAD clearance does not establish clearance on the actual field.
5. START runs the selected sequence. STOP interrupts it. At DONE, commands stay
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
- **Square:** load the experimental field preset, replacing all selections for
  that mode. It does not move the robot. Check every row before START.

| Row | B default | C default | Increment / range |
| --- | ---: | ---: | --- |
| Flywheel ticks/s | 1250 | 1250 | 50; 500-2500 |
| Feed seconds | 10 | 7.5 | 0.5; 0-20; 0 skips shooting |
| Travel cm | 60.96 | 115.93 | 5; 0-350; second leg only |
| Drive vector power | 0.20 | 0.45 | 0.05; 0.10-0.50 |
| Travel angle deg (+left) | +25.87 | -79.261 | Next whole degree, then 1; -90 to +90 |
| Intake + agitator power | 0.50 | 0.50 | 0.05; 0.10-1.00 |
| Start delay seconds | 0 | 15 | 0.5; 0-20, subject to budget |
| Calibration | 0: tested baseline | 1: MAT_TRIAL | 0 or 1 |
| Initial clearance cm | 0 | 20.32 | 5; 0-100; **B left, C forward** |

**Travel cm is the main/second leg, not the total distance of a two-leg route.**
Setting travel to zero does not remove a nonzero clearance leg. To shoot in
place, set **both Travel cm and Initial clearance cm to zero**. To test driving
without shooting, set Feed seconds to zero. All three zero leave delay/hold only;
hardware initialization still runs.

Angles describe translation relative to the starting chassis heading, not a
chassis turn. Positive means robot-left; negative robot-right. MAT_TRIAL adjusts
the motor-command angle to estimate the desired physical direction. Telemetry
shows both angles. It also changes duration by direction. Baseline preserves
the old `travel_cm / (129 * power)` timing and direct sine/cosine command.
MAT_TRIAL assumes unconfirmed original test settings: verify it with short
trials before relying on a field route.

## Primary field preset: Square in B

The default 60.96-cm baseline is a short test, not a full-field park. Square loads:

- MAT_TRIAL, power **0.25**, normal 10-second feed and zero delay;
- **63.50 cm left** clearance after shooting;
- then **227.25 cm at +11.775 degrees** to the near parking slot.

This avoids the floor-level frame bar in the nominal CAD. It budgets about
**24.19 s** including worst-case spinup and hold. Triangle restores the tested
short route, removing the clearance leg. Editing either leg changes the route;
check the geometry again after such edits.

## Supporting field preset: C

Use the offset-and-aim pose from the field guide: relative to the directly
centered end-wall position, shift about **6.07 cm toward the field centerline**
and aim about **3.014 degrees to robot-right** back at the cell. A rear corner
contacts the wall. Nominal flower clearance is only **2.37 cm**; check it on the
actual field and verify shooting after the cell is tipped upward.

Default sequence, ignoring small loop-transition delays:

| Time from START | Action |
| --- | --- |
| 0-11 s | All motor commands zero |
| 11-15 s | Flywheel spins; drive, intake and agitator remain off |
| 15-22.5 s | Feed for 7.5 s if speed qualified; remain stationary |
| After 22.5 s | Stop all shooting mechanisms; drive 20.32 cm forward to clear flower |
| Next | Travel 115.93 cm at -79.261 degrees (mostly right, slightly forward) |
| End | Hold zero outputs for 0.5 s, then DONE |

This uses power 0.45 and MAT_TRIAL; predicted total is about **25.8 s**. Neither
this power/direction combination nor the route has been validated on the robot.
The partner's hive tip is an assumption, not a detected event. Shooting into a
cell that has not tipped is not corrected automatically.

If the launcher is not ready at the end of the delay, C skips the shot and
waits through the selected feed slot before departing. Shooter INIT fallback
also retains that slot. If you deliberately set Feed seconds to zero, both
prespin and the feed slot disappear: departure starts after the selected delay.
With feeding enabled, delay cannot be reduced below four seconds.

## Shooting, memory and failure handling

The launcher qualifies at 96% of target for 0.25 seconds. Then launcher, main
intake and agitator run for the full feed duration; ordinary speed dips do not
pause feeding. No windmill feeder is used. Encoder target is ticks/s, not RPM;
feed time is commanded motor time, not a ball count.

B spins up after its delay, with a four-second timeout. Timeout skips the shot
and continues its selected route. C spends that spinup allowance inside the
last four seconds of its delay. Invalid velocity data stops either mode.

A, B and C remember settings separately while the Robot Controller app runs.
Start delay resets every INIT: zero for A/B, 15 for C. If C's remembered route
would exceed the budget with restored delay, all C defaults are restored with
a visible notice. App restart/reinstall loses memory. Triangle is not a recall
of the last successful match settings.

Edits are rejected if worst-case total reaches **26 seconds**. Both clearance
and main leg count toward this budget. An independent **27-second cutoff**
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
then the combined route. Confirm zero shooter output before driving. Repeat
with actual load, expected battery and the partner's route. Verify both alliance
placements. Do not equate computer tests or SDK compilation with a tested route.

Record: installed commit/build; date; battery; surface/load; A/B/C; measured start
center/heading; profile; flywheel/feed settings; delay; drive power; clearance cm;
main-leg cm/angle; actual forward/side displacement; final heading; endpoint;
shots and STOP result; partner parking slot; who verified the complete route.

Build/deployment instructions are in the
[package README](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/README.md).
