## TeamCode: what's in here

All of team 31452's robot code lives in this folder. New here? Start with
[docs/POLLEN_START_HERE.md](../../../../../../../../docs/POLLEN_START_HERE.md), and
[docs/ROBOT_CONNECTION.md](../../../../../../../../docs/ROBOT_CONNECTION.md) to set up your laptop.

### Live: PollenBot (this season's robot)

| Driver Hub name | File or folder | What it is |
| --- | --- | --- |
| **PollenBot** (TeleOp) | `BioBuzzStarterbotOpMode.java` | The match TeleOp |
| Odo ROBOT Shoot + Park, Odo Supporting Shooter Auto, Odo ROBOT Park, and the EMERGENCY Timed autos | `pollen/` | All the autonomous code; see `pollen/README.md` |
| Angled Primary Shooter (group Pollen Experimental) | `pollen/PollenAngledPrimaryShooter.java` | **Experimental:** the primary auto with an angled shot. Not yet tried on a field |
| LauncherTest (group Test) | `LauncherTestOpMode.java` | Runs PollenBot's launcher, intake and agitator on their own, for testing. Same buttons, INIT menu and feed rule as the TeleOp |
| Odometry Check (group Pedro) | `OdometryCheck.java` | Checks the Pinpoint odometry pods; steps in `docs/ODOMETRY_BASELINE_TEST.md` |
| Pedro Test (group Pedro) | `PedroLocalizationTest.java` | Shows Pedro Pathing's position on the Panels dashboard and follows a short test path |
| (not an OpMode) | `pedroPathing/Constants.java` | Robot measurements and settings for Pedro Pathing and the Pinpoint |
| (hidden) | `SensorGoBildaPinpoint.java` | goBILDA's Pinpoint sample, kept for reference |

Computer tests for `pollen/` are in `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/pollen/`:
run `PollenTests` from Android Studio.

### Old: Lox (last season's DECODE robot)

`lox/` holds the code for Lox, kept so it still runs. `BioBuzzOpMode` and `MecanumOpMode` still
appear in the Driver Hub list; the rest is hidden or helper classes. It uses different motor names
(`flywheel`, `leftDrive`, ...), so it does not work on PollenBot. Don't add new code there.

### Adding an OpMode

The easiest way is to copy a sample and make it your own. The samples are in the
FtcRobotController module (Project view → `FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples`).
Copy one into this folder, change its `@TeleOp(name = ...)`, and remove its `@Disabled` line so
it shows up on the Driver Hub. Then open a pull request (see ROBOT_CONNECTION.md).
