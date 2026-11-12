# Hive camera and garden practice

**Build pollen-hive-v1 - experimental.** Use the full [hive testing guide](../POLLEN_HIVE_TESTING.md) for setup, coordinates and troubleshooting. Keep the existing match autos available.

## 1. Camera test: NO MOTORS

- Rear camera, lens center about **14 inches** above tiles. USB-C camera to blue USB-A on Control Hub. Configure its name as `limelight`.
- Select **TEST Hive camera (NO MOTORS)**. INIT: D-pad chooses slot, pipeline and actual near-cell tag ID. START still runs no motors.
- **Triangle:** clear candidate. **Square:** five-second UP capture. **Circle:** five-second DOWN capture if visible. **Cross:** save profile. Hold the real hive still during capture.
- Validate after saving: tip 20 times, cover tag, show wrong tag, disconnect/reconnect. UP must only appear when this cell can accept a shot. DOWN or UNKNOWN means false.
- Repeat at each shooting position/alliance. Photograph mount and pipeline settings. A changed mount or camera configuration needs new calibration.

## 2. Choose an experimental auto

| Mode | What happens |
| --- | --- |
| EXPERIMENTAL Hive Secondary | Wait for near cell UP, shoot, park |
| EXPERIMENTAL Hive Primary + Garden | Shoot, stop flywheel, collect, return, wait/shoot, park |

**INIT controls:** D-pad up/down chooses a row, left/right changes it. Square saves settings; Triangle restores CAD defaults. START locks settings. Check the correct saved profile/tag.

| Execution row | Motors |
| --- | --- |
| 0 DRY (default every INIT) | All off; virtual launcher speed shows feed requests. Stationary check only. |
| 1 DRIVE | Drivetrain only. Shots skipped; tests the route and turns without balls. |
| 2 LIVE | Drive, intake, launcher and agitator active as requested |

Test in that order. Verify field clearances and agree the lane with the partner. Check Pinpoint READY before START. STOP ends the run. A fault latches all commands off.

<div class="page-break"></div>

## 3. Garden route and front intake

Start with **front intake against end wall; rear launcher/camera toward hive**. The diagram shows red; rotate the complete setup 180 degrees for blue.

<img src="../POLLEN_HIVE_GARDEN.svg" style="width:6.8in;max-height:3.8in">

- S→A clears wall; A→B reaches a turning point away from the midline.
- B turns -90 degrees. B→C travels front first toward garden.
- C turns to 0 degrees, D aligns the intake, D→E collects toward the wall.
- Back out E→D, clear the side wall at C, then C→A→S. Rear faces hive again.
- No turn at E. Body clearance is only **1 inch** to the two nearby walls. Confirm intake extensions and odometry error fit.
- All four ball centers require about **17.45 inches of centered intake width** at E. A narrower intake may collect fewer; verify actual reach.

## 4. Shooting and time checks

Feeding needs fresh confirmed UP, settled shooting pose and measured flywheel speed. Covering the tag or tipping away stops feeding. Returning to S discards old camera readiness. Already launched balls keep flying.

Both modes reserve estimated parking time and stop at **27 seconds**. Slow first shot or flywheel coast can cause **garden skipped**. After return, DOWN/UNKNOWN means wait until UP or departure time. Never change UNKNOWN into permission to shoot.

**Record:** profile/tag, INIT settings, pickup count, return error, cycle time, any false UP. Save settings/captures and pull the `PollenHive` robot log before more runs overwrite it. Computer simulations are not field validation.
