# PollenBot tested behavior on competition tiles

What the current PollenBot hardware has actually been measured or seen to do,
kept apart from what the code only predicts. Field geometry is in
[POLLEN_FIELD_LAYOUT.md](POLLEN_FIELD_LAYOUT.md); how the drive models are
built from these numbers is in [POLLEN_MAT_CALIBRATION.md](POLLEN_MAT_CALIBRATION.md).

As of build `pollen-mat-v9` (October 24, 2026), **only two autonomous settings
have run successfully on the robot: A's defaults and B's Triangle defaults.**
Every other route, the MAT_TRIAL drive model and the TeleOp INIT menu are
checked in software only. Add every new physical result to this file.

## Status by feature

| Feature | Settings | Physical status | Evidence |
| --- | --- | --- | --- |
| A: Pollen ROBOT Park | 2.00 s forward at power 0.20, delay 0 | Run successfully | Team report; endpoint not recorded |
| B: Shoot + Park, Triangle | 1250 ticks/s, feed 10 s, 60.96 cm at +25.87 deg, power 0.20, baseline model | Run successfully | Team report; endpoint and shots not recorded |
| PollenBot TeleOp, untouched menu | launcher 1250, feed at 1200 ticks/s or 3 s after R1, drive 100% | Feed stopped working Oct 24 under the old speed-only gate; v8's speed-or-timer feed not yet run | See "Feeder stall, October 24" |
| Launcher spin-up | 3 s before feeding | Team-measured | Basis for the fixed spin-up in auto and TeleOp |
| Leg 3 defaults | B Square 172.47 cm at +17 deg; C 97.96 cm at -81.85 deg | Team-chosen on the field, Oct 24 | Endpoints not recorded here; the drive model predicts different endpoints |
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
| Feed starts | after the Spin-up row (3 s), whatever the measured speed | at 96% of target or 3 s after R1, whichever is first | 3 s is team-measured |
| Feed power | intake + agitator 0.50 | trigger, +0.50 while firing | Code |
| Feed duration | B 10 s, C 7.5 s | while R1 held | Pollen per second not measured |

### Feeder stall, October 24

The launcher spun in auto and TeleOp but the feeder never ran. Both used to wait
for the encoder to read at least 1200 ticks/s (96% of target); TeleOp's check
came from the original StarterBot code. The likeliest cause is the measured
speed no longer reaching 1200: a loose launcher encoder cable, a low battery, or
a raised TeleOp INIT speed. Build `pollen-mat-v5` feeds on fixed times instead,
and TeleOp telemetry flags a measured speed near 0. Check "Launcher target /
measured" on the next run and record what it shows.

On `pollen-mat-v5` the launcher and intake ran but the agitator still did not.
The agitator replaced a windmill servo that ran at full power while shooting
(PR #15 commented out `agitator.setPower(1)`), and since then it had only
received the intake's 0.5 command. Build `pollen-mat-v6` runs the agitator at its
own feed power, default 1.0, in auto and TeleOp, and shows "Intake / agitator
command" on the screen. If that line reads 1.00 and the agitator still does not
turn, check its wiring, port and configuration name (`agitator`).

Because the agitator, not the speed check, was still failing after v5, build
`pollen-mat-v8` brings the speed check back in TeleOp only, as an early start:
feeding begins at 96% of target speed or after 3 s, whichever is first, and the
screen says which. If it says "up to speed" on every shot, the encoder reads
correctly. If it always says "spin-up timer", the launcher never reached 1200
ticks/s, which may be why the original strict check failed.

Not measured yet: actual speed reached in 3 s, pollen launched per second, shot
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
| C timing | finish before the 27 s hard stop | planned 25.4 s; 0.6 s under the editor's 26 s limit |

A route is usable once repeated runs land well inside these margins with the
real load and battery.

## Highest-value next measurements

Odometry and Pedro Pathing will replace timed driving, so MAT_TRIAL is frozen:
run the current routes to confirm they are safe to use until then, but do not
spend field time refining the drive model.

1. **Launcher**: TeleOp's "Feed" line on each shot ("up to speed" or "spin-up
   timer"), measured speed after the 3 s spin-up, then pollen per 10 s feed,
   from B's start and C's start. This carries over to every future autonomous.
2. **B Square and C, full routes as installed**: does the robot end partly in
   the loading zone and off the wall, without touching the hive frame or a
   flower? Record the endpoint and heading.
3. **Cycle timing**, once the mechanisms exist: seconds to shoot a full load,
   seconds to collect one, and driving time between the shooting spot, the
   flower and the parking zone. See
   [what carries over](POLLEN_AUTONOMOUS_EXPLAINED.md#what-carries-over-to-odometry-and-pedro).
4. **Footprint and C placement**: real width and length including intake and
   bumpers, the height of the part that touches the wall, and whether the real
   robot fits between the event field's flower and the midline with C's
   2.6 cm-per-side nominal gaps.
5. **Loop rate**: note any `LOOP STALL WHILE DRIVING` stop. The code now stops
   on a pause over 0.25 s while driving, so a stall shows up instead of
   silently moving the robot off its route.

Record one row per run:

| Date | Build | Mode / leg | Profile | Power | Requested cm / deg | Seconds | Forward cm | Sideways cm | Heading change deg | Battery V | Load | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| | | | | | | | | | | | | |

If a route misses, adjust that leg's distance or angle in the INIT menu and
record the new setting here; do not refit `PollenDriveCalibration.java`.
