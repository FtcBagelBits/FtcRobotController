# PollenBot tested behavior on competition tiles

What the current PollenBot hardware has actually been measured or seen to do,
kept apart from what the code only predicts. Field geometry is in
[POLLEN_FIELD_LAYOUT.md](POLLEN_FIELD_LAYOUT.md); how the drive models are
built from these numbers is in [POLLEN_MAT_CALIBRATION.md](POLLEN_MAT_CALIBRATION.md).

As of build `pollen-mat-v3` (October 24, 2026), **only two autonomous settings
have run successfully on the robot: A's defaults and B's Triangle defaults.**
Every other route, the MAT_TRIAL drive model and the TeleOp INIT menu are
checked in software only. Add every new physical result to this file.

## Status by feature

| Feature | Settings | Physical status | Evidence |
| --- | --- | --- | --- |
| A: Pollen ROBOT Park | 2.00 s forward at power 0.20, delay 0 | Run successfully | Team report; endpoint not recorded |
| B: Shoot + Park, Triangle | 1250 ticks/s, feed 10 s, 60.96 cm at +25.87 deg, power 0.20, baseline model | Run successfully | Team report; endpoint and shots not recorded |
| PollenBot TeleOp, untouched menu | launcher 1250 / feed gate 1200, drive 100% | Team's working TeleOp | Same numbers as before the INIT menu existed |
| B, Square field route | 3 legs, MAT_TRIAL, power 0.25 | Not run | Software tests and CAD route checker only |
| C: Supporting Shooter | flush start, 2 legs, MAT_TRIAL, power 0.45 | Not run | Software tests and CAD route checker only |
| MAT_TRIAL drive model | interpolated from the trials below | Not validated | Built from trials whose setup is partly unknown |
| TeleOp INIT menu (changed values) | other launcher speeds, drive % | Not run | Software tests only |

## Robot facts used by the code

| Fact | Value | Status |
| --- | --- | --- |
| Footprint | 18 x 18 in (45.72 cm), modeled as a filled square | Team report; intake and bumpers not measured |
| Drivetrain | 4-wheel mecanum; `frontLeft` REVERSE, `frontRight` FORWARD, `backLeft` FORWARD, `backRight` REVERSE; BRAKE; no encoders used | Code; matches the team's tested TeleOp |
| Mixer | wheel = forward +/- strafe +/- turn, normalized if any exceeds 1 | Code |
| Launcher | `pollenLauncher`, REVERSE, RUN_USING_ENCODER, PIDF 40 / 0 / 0 / 12.5 | Code (goBILDA StarterBot values) |
| Launcher units | ticks/s; the code comment converts 1250 to about 2678 RPM (28 ticks per revolution) | Code comment |
| Feed | `intake` FORWARD and `agitator` REVERSE, BRAKE | Code |
| Launcher position | assumed on the robot centerline | Assumption; not measured |
| Localization | none in Pollen code: timed, open-loop moves; no heading correction | Code |

## Drive measurements on competition tiles

### Forward, known power and time

| Power | Seconds | Measured cm | cm / (power x s) |
| ---: | ---: | ---: | ---: |
| 0.25 | 1.0 | 32.6 | 130.40 |
| 0.25 | 2.0 | 68 | 136.00 |
| 0.25 | 3.0 | 105 | 140.00 |
| 0.50 | 1.5 | 113 | 150.67 |

The coefficient grows with run time, consistent with acceleration at the start
of each move. Least squares through zero at 0.25 power gives **138.17**; the
single 0.50 point gives **150.67**. Battery, robot load and repetitions were not
recorded. Code: `PollenDriveCalibration.FORWARD_MAT_COEFFICIENT` and
`FORWARD_HALF_POWER_COEFFICIENT`.

### Shoot-mode requested moves

| Requested | Measured forward | Measured sideways | Distance | Direction |
| --- | ---: | ---: | ---: | ---: |
| 100 cm forward | 107 cm | not reported | 107 cm | 0 deg |
| 100 cm sideways | not reported | 80 cm | 80 cm | 90 deg |
| 75 cm at 45 deg | 54 cm | 31 cm | 62.3 cm | 29.86 deg |

Unknown for these three: drive power, installed code and coefficient, battery,
load, left or right strafe, repetitions and heading drift. The team marked the
settings as "different or uncertain". What they do show:

- If those runs used the 129 baseline, forward travelled about 7% farther than
  requested and sideways about 20% less.
- A 45 deg command moved at about 30 deg. The sideways part comes up short, so
  more drive time alone keeps the wrong direction.

MAT_TRIAL turns these three points into a correction for both angle and time,
assuming the old 129 model was installed and that left and right behave the
same. Code: `PollenDriveCalibration.move()`.

### What the models predict at power 0.20 (not measured)

| Request | Baseline (129) | MAT_TRIAL |
| --- | --- | --- |
| 100 cm forward | 3.876 s at 0 deg | 3.622 s at 0 deg |
| 100 cm sideways | 3.876 s at 90 deg | 4.845 s at 90 deg |
| 75 cm at +45 deg | 2.907 s at +45 deg | 3.534 s at +56.33 deg |
| A default, 2 s forward | 51.6 cm | 55.3 cm (forward fit, extrapolated below 0.25) |

## Shooter behavior

| Setting | Auto (B and C) | TeleOp | Measured? |
| --- | --- | --- | --- |
| Launcher target | 1250 ticks/s (INIT row) | 1250 ticks/s (INIT menu) | Used in matches; exit speed not measured |
| Feed allowed at | 96% of target (1200), held 0.25 s | above 96% of target (1200) | Code |
| Spin-up limit | 4 s, then skip the shot | none | Spin-up time not measured |
| Feed power | intake + agitator 0.50 | trigger, +0.50 while firing | Code |
| Feed duration | B 10 s, C 7.5 s | while R1 held | Pollen per second not measured |

Not measured yet: time to reach 1200 ticks/s, pollen launched per second, shot
spread, and whether 4 pre-loads tip the hive from either start. The field's own
calibration (3 nectar + 2 pollen holds, one more tossed pollen tips) is a field
check, not a launch count.

## Margins that set the accuracy we need

| Route | What must be true | Nominal margin |
| --- | --- | --- |
| B Square and C parking | end partly on the tape and off the side wall | about 14 cm each way, roughly 12% of the 110-117 cm lateral travel |
| C start | 18 in robot between the flower and the midline | 2.6 cm per side (5.2 cm total slack) |
| B Square leg 3 | clear the side-wall flower | 21.6 cm |
| C leg 1 | pass the flower going forward | 2.6 cm |

A route is usable once repeated runs land well inside these margins with the
real load and battery.

## Highest-value next measurements

1. **Strafe left and right** at 0.25 and 0.45 power, 1-3 s, feed set to 0:
   measure forward and sideways separately and the final heading. MAT_TRIAL's
   biggest assumptions are sideways distance and left/right symmetry.
2. **B Square, legs one at a time** (other legs set to 0 cm), then the full
   route: lateral endpoint and heading.
3. **C legs** at power 0.45, the same way.
4. **Launcher**: time to 1200 ticks/s, then pollen per 10 s feed, from B's
   start and C's start.
5. **Footprint**: real width and length including intake and bumpers, and the
   height of the part that touches the wall.

Record one row per run:

| Date | Build | Mode / leg | Profile | Power | Requested cm / deg | Seconds | Forward cm | Sideways cm | Heading change deg | Battery V | Load | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| | | | | | | | | | | | | |

After adding data, update the constants in `PollenDriveCalibration.java` and
their asserted values in `tools/pollen-tests/PollenMatSupportTest.java`, then
rerun `bash tools/test-pollen-sequence.sh`. Do not fit a new correction on top
of runs made with an unknown earlier correction.
