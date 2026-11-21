/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pollen.IsHiveReadyForShot;
import org.firstinspires.ftc.teamcode.pollen.PollenConfig;
import org.firstinspires.ftc.teamcode.pollen.PollenHiveAim;
import org.firstinspires.ftc.teamcode.pollen.PollenHiveCamera;
import org.firstinspires.ftc.teamcode.pollen.PollenHiveSettings;
import org.firstinspires.ftc.teamcode.pollen.PollenFacing;
import org.firstinspires.ftc.teamcode.pollen.PollenHiveStore;
import org.firstinspires.ftc.teamcode.pollen.PollenHiveTrace;
import org.firstinspires.ftc.teamcode.pollen.PollenOdometryDrive;
import org.firstinspires.ftc.teamcode.pollen.PollenOdometryHardware;
import org.firstinspires.ftc.teamcode.pollen.PollenRunLog;
import org.firstinspires.ftc.teamcode.pollen.PollenTeleOpRecorder;
import org.firstinspires.ftc.teamcode.pollen.PollenTeleOpTuning;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot with Mecanum
 * Wheels for the 2026-2027 FIRST® Tech Challenge. On top of a mecanum wheel drivetrain, it uses
 * one motor driving an intake roller, two servos which pull elements out of corners, and a high-speed
 * launcher motor.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "PollenBot", group = "1 Match")
//@Disabled
public class BioBuzzStarterbotOpMode extends OpMode {

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private DcMotor agitator = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;


    /*
     * These two variables are used to control the velocity of the launcher motor.
     * They are both in encoder ticks per second. The motors we use in the FIRST Tech Challenge
     * have encoders with a resolution of 28 ticks per revolution. We can convert this to RPM
     * by dividing the value by 28, to get to revolutions per second, before multiplying by 60
     * to get revolutions per minute.
     * We pass the target velocity variable to our motor to set the goal. Feeding starts when the
     * launcher reads 96% of target or when the spin-up time runs out, whichever is first, so a
     * low encoder reading can delay feeding by at most the spin-up time but never block it.
     * That feed rule is written once, in pollen/PollenFeedRule.java, and autonomous and
     * LauncherTest use the same one. The default speed is shared too (pollen/PollenConfig.java).
     */
    public final int LAUNCHER_TARGET_VELOCITY = PollenConfig.LAUNCHER_TARGET_VELOCITY; //1180 = 2529 RPM (was 1250)
    public final double LAUNCHER_SPINUP_SECONDS = PollenConfig.LAUNCHER_SPINUP_SECONDS; // 3 s
    // Hold Square to run the launcher BACKWARDS slowly and push a jammed ball back out.
    // 300 RPM = 300 x 28 / 60 = 140 ticks per second.
    public final int LAUNCHER_REVERSE_VELOCITY = PollenConfig.LAUNCHER_REVERSE_VELOCITY; //140 = 300 RPM
    // While feeding the launcher, this much is added to the intake power from the triggers,
    // so the intake keeps pushing balls toward the launcher. (Autonomous has its own
    // "Intake power while feeding" row, which also defaults to 0.5.)
    public final double INTAKE_FEED_BOOST = PollenConfig.INTAKE_FEED_BOOST; // 0.5
    // The launcher's MEASURED speed in ticks per second (the target is tuning.launcherVelocity()).
    // We read it ONCE in each loop, so the feed decision and the Driver Station screen always
    // use the same number.
    double launcherMeasuredVelocity;
    private boolean reversing = false;
    private final ElapsedTime launchTimer = new ElapsedTime();
    private boolean launching = false;
    private boolean feeding = false;   // latched on until R1 is released
    private String feedStartedBy = "";
    private double feedStartedAt = 0;

    /*
     * INIT menu (like the Pollen autonomous menus): before START the driver can raise or lower the
     * launcher speed and spin-up time and turn down the drive power. The numbers above are the defaults, so an
     * untouched menu drives and launches exactly as before. Settings lock at START and are kept
     * for the next INIT until the Robot Controller app restarts; Triangle restores the defaults.
     */
    private static PollenTeleOpTuning.Snapshot remembered;
    private PollenTeleOpTuning tuning;


    /*
     * These four variables store the power we need to apply to the motors. In other cases, we may
     * choose to declare these variables inside the mecanumDrive() function, instead we declare them
     * here so that we can access them in our main loop for telemetry.
     */
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    // Create a variable to set to the intake.
    double intakePower;
    private double agitatorPower;

    private static final String SERVO_NAME    = "pollenDrainer";

    private Servo armServo;

    // Tracks where we told the servo to go
    private double armPosition;
    private boolean drainerDown;

    // Used so holding a button only acts once, not every loop
    private boolean aWasPressed = false;
    private boolean squareWasPressed = false;
    private boolean upWasPressed = false;
    private boolean downWasPressed = false;

    /*
     * RECORDING. Every TeleOp run is written to a file on the Control Hub (see
     * pollen/PollenTeleOpRecorder.java and docs/POLLEN_LOGGING.md): what the driver did, what
     * every motor was told to do, the launcher speed, the battery and, if the robot has its
     * Pinpoint odometry, where the robot was. Match 2 (v39): no rehearsal marks from the gamepad;
     * L1 is the unjam button and Triangle does nothing after START. The recorder's mark support stays
     * (PollenTeleOpRecorder) for a practice mode that wants it.
     *
     * Recording never changes how the robot drives. If anything about it fails (no room on the
     * Hub, no Pinpoint, a write error), TeleOp carries on and the screen says what is missing.
     */
    /*
     * AIM (Circle, v36): turn in place so the launcher points at the middle of the hive cell the
     * Limelight sees nearest the launcher's line; see pollen/PollenHiveAim.java. Every aim is an
     * AIM row in the recording. Circle again aims again from where the robot is; the turn stick cancels it. The calibration mode
     * ("Calibration Hive Aim") also aims by odometry from the primary start: D-pad LEFT = camera,
     * RIGHT = odometry.
     */
    protected boolean aimCalibration() { return false; }
    private PollenHiveCamera camera;             // stays null without a working Limelight
    private String cameraProblem = "";
    private IsHiveReadyForShot.Mount mount;
    private final PollenHiveAim aim = new PollenHiveAim();
    private PollenHiveAim.Source aimSource = PollenHiveAim.Source.CAMERA;
    private IsHiveReadyForShot.Frame lastFrame;
    private double lastFrameStamp = Double.NaN, lastFrameAt = Double.NEGATIVE_INFINITY;
    private double routeX = Double.NaN, routeY = Double.NaN, routeH = Double.NaN, previousH = Double.NaN, previousHAt, headingRate;
    private boolean circleWasPressed, aimLeftWas, aimRightWas;
    private double aimTurn;

    private PinpointLocalizer odometry;          // stays null if there is no Pinpoint
    private GoBildaPinpointDriver pinpoint;
    private String odometryProblem = "";
    private PollenHiveTrace recordingFile;       // stays null if recording could not start
    private PollenTeleOpRecorder recorder;
    private final PollenTeleOpRecorder.Sample sample = new PollenTeleOpRecorder.Sample();
    private final ElapsedTime runTimer = new ElapsedTime();
    private String storageLine = "", storageWarning = "", recordingProblem = "";
    private double batteryVolts = Double.NaN, batteryReadAt = Double.NEGATIVE_INFINITY;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
        agitator = hardwareMap.get(DcMotor.class, "agitator");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "pollenLauncher");
        armServo = hardwareMap.get(Servo.class, SERVO_NAME);
//        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
//        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        agitator.setDirection(DcMotor.Direction.REVERSE);
        launcher.setDirection(DcMotor.Direction.REVERSE);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        agitator.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
//        leftIntakeServo.setPower(0);
//        rightIntakeServo.setPower(0);

        /*
         * Much like our drivetrain motors, we set the right intake servo to reverse so that both
         * servos work to pull elements into the intake.
         */
//        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        tuning = new PollenTeleOpTuning(LAUNCHER_TARGET_VELOCITY, LAUNCHER_SPINUP_SECONDS,
                PollenConfig.AGITATOR_FEED_POWER, remembered, true);

        /*
         * Odometry is OPTIONAL here: TeleOp only writes the robot's position into the recording.
         * Like the autonomous modes, the Pinpoint recalibrates now, so keep the robot still
         * during INIT. With no Pinpoint (or a broken one) TeleOp works exactly as before.
         */
        try {
            if (aimCalibration()) {
                // The calibration mode starts at the primary shooter start: zero the Pinpoint there.
                pinpoint = PollenOdometryHardware.device(hardwareMap);
                odometry = PollenOdometryHardware.openZeroed(hardwareMap, pinpoint);
            } else {
                odometry = PollenOdometryHardware.open(hardwareMap);
                pinpoint = PollenOdometryHardware.device(hardwareMap);
            }
        } catch (Exception ex) {
            odometry = null;
            pinpoint = null;
            odometryProblem = "no working Pinpoint";
            RobotLog.ii("PollenTeleOp", "No odometry in this run: %s", ex.toString());
        }
        // Circle aims with the Limelight, using the same camera calibration as the hive autos.
        try {
            PollenHiveSettings.Values cameraSettings = new PollenHiveSettings.Values(PollenHiveSettings.CAMERA, false);
            PollenHiveStore.load(PollenHiveStore.CAMERA, cameraSettings);
            mount = IsHiveReadyForShot.Mount.from(cameraSettings);
            camera = new PollenHiveCamera(hardwareMap, (int) cameraSettings.get("pipeline"));
        } catch (Exception ex) {
            camera = null;
            cameraProblem = ex.toString();
        }
        // How much room the Control Hub has for recordings. Read once: it looks through folders.
        try {
            PollenRunLog.Storage storage = PollenHiveStore.storage();
            storageLine = storage.line();
            storageWarning = storage.warning();
        } catch (Exception ex) {
            storageLine = "unknown (" + ex + ")";
        }

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // Before START, D-pad edits the menu without moving the drainer.
        tuning.update(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left, gamepad1.dpad_right,
                gamepad1.triangle);
        remembered = tuning.snapshot();
        telemetry.addData("Build", PollenConfig.BUILD);
        // Same order as the menu: rows to check or maybe change first, never-changed rows last.
        telemetry.addData(marker(PollenTeleOpTuning.Row.LAUNCHER) + "Launcher ticks/s", "%.0f (default %d)",
                tuning.launcherVelocity(), LAUNCHER_TARGET_VELOCITY);
        telemetry.addData(marker(PollenTeleOpTuning.Row.DRIVE) + "Drive power", "%.0f%% (default 100%%)",
                tuning.driveScale() * 100);
        telemetry.addData(marker(PollenTeleOpTuning.Row.AIM_LEFT) + "Circle aim target in left of center", "%.1f (default %.1f; - = right)",
                tuning.aimTargetLeftIn(), PollenConfig.AIM_TARGET_LEFT_IN);
        telemetry.addData(marker(PollenTeleOpTuning.Row.SPINUP) + "Spin-up before feed", "%.1f s (default %.1f s)",
                tuning.spinupSeconds(), LAUNCHER_SPINUP_SECONDS);
        telemetry.addData(marker(PollenTeleOpTuning.Row.AGITATOR) + "Agitator power while feeding", "%.2f (default %.2f)",
                tuning.agitatorPower(), PollenConfig.AGITATOR_FEED_POWER);
        telemetry.addData(marker(PollenTeleOpTuning.Row.DRAINER_UP) + "Drainer up position", "%.2f (+/- 0.01)", tuning.drainerUpPosition());
        telemetry.addData(marker(PollenTeleOpTuning.Row.DRAINER_DOWN) + "Drainer down position", "%.2f (+/- 0.01)", tuning.drainerDownPosition());
        telemetry.addData(marker(PollenTeleOpTuning.Row.DRAINER_STEP) + "Drainer step", "%.3f (+/- 0.002)", tuning.drainerStep());
        telemetry.addData("Controls", "D-pad UP/DOWN selects; LEFT/RIGHT edits; Triangle = defaults");
        telemetry.addData("Settings", tuning.isDefault() ? "DEFAULTS" : "CHANGED - check before START");
        telemetry.addData("Recording", "this run is saved on the Hub");
        telemetry.addData("Circle = aim at the hive", camera == null ? "NO CAMERA (" + cameraProblem + ")" : "Limelight " + camera.status
                + (aimCalibration() ? ". CALIBRATION: start at the PRIMARY shooter start; D-pad LEFT camera / RIGHT odometry" : ""));
        telemetry.addData("Recorded position", odometry == null ? "none (" + odometryProblem + "); the run is still recorded"
                : "Pinpoint " + PollenOdometryHardware.status(pinpoint) + " - keep the robot still during INIT");
        telemetry.addData("Hub storage", storageLine);
        if (!storageWarning.isEmpty()) telemetry.addData("STORAGE WARNING", storageWarning);
    }

    private String marker(PollenTeleOpTuning.Row row) {
        return tuning.selected() == row ? "> " : "  ";
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        tuning.lock();
        runTimer.reset();
        startRecording();
        // Apply the selected up position only after START.
        armPosition = tuning.drainerUpPosition();
        drainerDown = false;
        aWasPressed = gamepad1.a;
        squareWasPressed = gamepad1.square;
        upWasPressed = gamepad1.dpad_up;
        downWasPressed = gamepad1.dpad_down;
        armServo.setPosition(armPosition);
        circleWasPressed = gamepad1.circle;
        aim.setTargetLeftIn(tuning.aimTargetLeftIn());
        aimLeftWas = gamepad1.dpad_left;
        aimRightWas = gamepad1.dpad_right;
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * Here we call a function called mecanumDrive. The mecanumDrive function takes the input from
         * the joysticks, and applies power to the drive motors to move the robot as requested
         * by the driver. Moving the left joystick forwards/back moves all motors forwards/back,
         * moving the right joystick left/right rotates the robot clockwise or counterclockwise,
         * and moving the left joystick left moves the motors in the right way to create a sideways
         * "strafe" movement. Combinations of these inputs can be used to create more complex maneuvers.
         * Note, moving the joystick forward on most gamepads results in a negative signal, so
         * we invert it before passing it to the function.
         */
        updatePose();
        readCamera();
        aimLoop();
        // While Circle's aim is turning, it owns the turn; the left stick still drives.
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, aim.active() ? aimTurn : gamepad1.right_stick_x);

        // Read the launcher's speed sensor (its encoder) once for this loop.
        launcherMeasuredVelocity = launcher.getVelocity();

        // D-pad: nudge the position one STEP, only on the moment it is first pressed
        boolean upIsPressed = gamepad1.dpad_up;
        if (upIsPressed && !upWasPressed) {
            armPosition += tuning.drainerStep();
        }
        upWasPressed = upIsPressed;

        boolean downIsPressed = gamepad1.dpad_down;
        if (downIsPressed && !downWasPressed) {
            armPosition -= tuning.drainerStep();
        }
        downWasPressed = downIsPressed;

        // A button: toggle, but only on the moment it is first pressed
        boolean aIsPressed = gamepad1.a;
        if (aIsPressed && !aWasPressed) {
            drainerDown = !drainerDown;
            armPosition = drainerDown ? tuning.drainerDownPosition() : tuning.drainerUpPosition();
        }
        aWasPressed = aIsPressed;

        // Square (v44): drainer UP, only on the moment it is first pressed. Cross still toggles. Raising
        // the drainer hands the intake back to the triggers and lets L1 unjam (L1 needs the drainer up).
        boolean squareIsPressed = gamepad1.square;
        if (squareIsPressed && !squareWasPressed) {
            drainerDown = false;
            armPosition = tuning.drainerUpPosition();
        }
        squareWasPressed = squareIsPressed;

        // Keep the position inside the servo's 0.0 - 1.0 range
        armPosition = Math.max(0.0, Math.min(1.0, armPosition));

        drainerDown = tuning.drainerIsDown(armPosition, drainerDown);

        // Send the servo to the chosen position
        armServo.setPosition(armPosition);

        launch();

        double triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
        // L1 (unjam): the triggers may only push balls OUT, never feed toward the launcher.
        if (gamepad1.left_bumper) triggerPower = Math.min(triggerPower, 0);
        agitatorPower = feeding ? tuning.agitatorPower() : triggerPower;
        if (!feeding && agitatorPower > 0) agitatorPower *= 0.5;
        // Draining runs the intake roller at full inward power. The agitator still needs driver input.
        intakePower = drainerDown ? 1.0 : Math.min(1.0, triggerPower + (feeding ? INTAKE_FEED_BOOST : 0));
        agitator.setPower(agitatorPower);
        intake.setPower(intakePower);
//        leftIntakeServo.setPower(intakePower);
//        rightIntakeServo.setPower(intakePower);

        /*
         * Show motor powers on the Driver Station via telemetry.
         */
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("Triggers", "left (%.2f), right (%.2f)", gamepad1.left_trigger, gamepad1.right_trigger);
        telemetry.addData("Launcher target / measured", "%.0f / %.0f ticks/s%s",
                tuning.launcherVelocity(), launcherMeasuredVelocity,
                launching && launcherMeasuredVelocity < 50 ? "  (measured ~0: check launcher encoder cable)" : "");
        telemetry.addData("Feed", reversing ? "REVERSING launcher (L1) - no feed"
                : gamepad1.left_bumper ? "Raise drainer (Square) to unjam with L1"
                : !launching ? "off (hold R1; L1 = reverse to clear a jam)"
                : feeding ? String.format("FEEDING (started at %.1f s: %s)", feedStartedAt, feedStartedBy)
                : "spinning up");
        telemetry.addData("Drainer", "%s (%.3f)", drainerDown ? "DOWN - intake ON" : "UP", armPosition);
        telemetry.addData("Intake / agitator command", "%.2f / %.2f",
                intakePower, agitatorPower);
        telemetry.addData("Drive power", "%.0f%%", tuning.driveScale() * 100);
        telemetry.addData("Aim (Circle)", aim.active() ? aimScreen : aim.lastResult());
        if (aimCalibration()) {
            PollenHiveAim.Bearing b = PollenHiveAim.bearing(freshFrame(), mount, aim.targetLeftIn());
            telemetry.addData("CALIBRATION source", aimSource + " (D-pad LEFT camera / RIGHT odometry)");
            telemetry.addData("Camera sees", b == null ? "no hive tags" : String.format("cell %d (%d tags): %.1f deg %s, %.0f in",
                    b.cluster, b.tags, Math.abs(b.degrees), b.degrees >= 0 ? "right" : "left", b.distanceIn));
            double sideways = PollenHiveAim.sidewaysToCenter(freshFrame(), mount, routeY, routeH);
            telemetry.addData("Camera sideways correction", Double.isFinite(sideways) ? String.format(
                    "now %.2f in; to center this cell %.2f in (valid with the robot centered on the cell)",
                    mount.sidewaysCorrectionIn, sideways) : "needs hive tags and the Pinpoint");
            telemetry.addData("Odometry", Double.isFinite(routeX) ? String.format("%.1f in away, %.1f in left, heading %.1f; aim %.1f deg",
                    routeX, routeY, Math.toDegrees(routeH), PollenHiveAim.odometryAimDegrees(routeX, routeY, aim.targetLeftIn())) : "none");
        }

        // Last, after every motor has its command: write this loop to the recording.
        record();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        logAim(aim.cancel(runTimer.seconds(), routeH, "stop"));
        try { if (camera != null) camera.stop(); } catch (RuntimeException ignored) { }
        // Finish the recording so the file is complete when it is downloaded.
        try {
            if (recorder != null) recorder.stop(runTimer.seconds(), System.currentTimeMillis(), sample);
            if (recordingFile != null) recordingFile.close();
            if (recorder != null) RobotLog.ii("PollenTeleOp", "STOP t=%.1f rehearsals=%d", runTimer.seconds(), recorder.rehearsals());
        } catch (RuntimeException ignored) {
            // A recording problem must never get in the way of stopping.
        }
    }

    /*
     * START: open this run's file and write every setting into it, so the run can be understood
     * later without guessing what the INIT menu said.
     */
    private void startRecording() {
        try {
            recordingFile = PollenRunLog.open(PollenHiveStore.folder(), PollenRunLog.Kind.TELEOP, System.currentTimeMillis());
        } catch (Exception ex) {
            recordingFile = null;
            recordingProblem = String.valueOf(ex.getMessage());
        }
        recorder = new PollenTeleOpRecorder(recordingFile);
        try {
            updatePose();
            fillSample();
            String settings = "build=" + PollenConfig.BUILD + " installed=" + PollenHiveStore.installed()
                    + " opmode=" + (aimCalibration() ? "PollenAimCalibration" : "PollenBot") + " launcher=" + tuning.launcherVelocity()
                    + " spinup=" + tuning.spinupSeconds() + " agitator=" + tuning.agitatorPower()
                    + " drive=" + tuning.driveScale() + " settings=" + (tuning.isDefault() ? "DEFAULTS" : "CHANGED")
                    + " reverse=" + LAUNCHER_REVERSE_VELOCITY + " feedBoost=" + INTAKE_FEED_BOOST
                    + " facing=" + PollenFacing.logName() + " pinpoint=" + (odometry == null ? "none" : sample.odometry.replace(' ', '_'))
                    + " battery=" + batteryVolts + " storage=[" + storageLine + "]"
                    + " camera=" + (camera == null ? "none" : "open") + " aim_tolerance_deg=" + PollenHiveAim.TOLERANCE_DEG
                    + " aim_target_left_in=" + tuning.aimTargetLeftIn()
                    + " lensHeight=" + (mount == null ? "NaN" : mount.lensHeightIn) + " tilt=" + (mount == null ? "NaN" : mount.tiltUpDeg)
                    + " sideways=" + (mount == null ? "NaN" : mount.sidewaysCorrectionIn);
            recorder.start(System.currentTimeMillis(), settings, sample);
            // The robot log gets the same line, so robotControllerLog.txt also shows the settings.
            RobotLog.ii("PollenTeleOp", "START %s recording=%s", settings,
                    recordingFile == null ? "OFF: " + recordingProblem : recordingFile.fileName);
        } catch (RuntimeException ex) {
            recordingProblem = ex.toString();
        }
    }

    /* Copy this loop's inputs, motor commands and sensor readings into the sample. */
    private void fillSample() {
        sample.stickForward = -gamepad1.left_stick_y;
        sample.stickStrafe = gamepad1.left_stick_x;
        sample.stickTurn = gamepad1.right_stick_x;
        sample.triggerIn = gamepad1.right_trigger;
        sample.triggerOut = gamepad1.left_trigger;
        sample.r1 = gamepad1.right_bumper;
        sample.l1 = gamepad1.left_bumper;
        sample.wheels[0] = leftFrontPower;
        sample.wheels[1] = rightFrontPower;
        sample.wheels[2] = leftBackPower;
        sample.wheels[3] = rightBackPower;
        sample.intake = intakePower;
        sample.agitator = agitatorPower;
        sample.launcherTarget = reversing ? -LAUNCHER_REVERSE_VELOCITY : launching ? tuning.launcherVelocity() : 0;
        sample.launcherMeasured = launcherMeasuredVelocity;
        sample.launching = launching;
        sample.feeding = feeding;
        sample.reversing = reversing;

        // Battery: each reading is a message to the hub, so once a second is plenty.
        if (runTimer.seconds() - batteryReadAt >= 1) {
            batteryReadAt = runTimer.seconds();
            double volts = Double.NaN;
            try {
                for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                    double v = sensor.getVoltage();
                    if (v > 0 && (Double.isNaN(volts) || v < volts)) volts = v;
                }
            } catch (RuntimeException ex) {
                volts = Double.NaN; // no reading this second
            }
            batteryVolts = volts;
        }
        sample.batteryVolts = batteryVolts;

        // Position, if there is a Pinpoint. If reading it ever fails, stop asking: TeleOp goes on.
        if (odometry == null) {
            sample.odometry = odometryProblem.isEmpty() ? "NO PINPOINT" : "NO PINPOINT: " + odometryProblem;
            sample.x = sample.y = sample.headingDegrees = Double.NaN;
            sample.rawX = sample.rawY = sample.rawHeading = Double.NaN;
            return;
        }
        try {
            Pose pose = odometry.pose(); // updated once per loop in updatePose()
            // Route directions, the same ones every autonomous uses (see pollen/PollenFacing.java).
            double[] route = PollenFacing.routePose(pose.x(), pose.y(), pose.heading());
            sample.odometry = PollenOdometryHardware.status(pinpoint);
            sample.x = route[0];
            sample.y = route[1];
            sample.headingDegrees = Math.toDegrees(route[2]);
            sample.rawX = pose.x();
            sample.rawY = pose.y();
            sample.rawHeading = pose.heading();
        } catch (RuntimeException ex) {
            odometry = null;
            odometryProblem = "read failed: " + ex;
        }
    }

    /* Every loop: write a row if it is time. No rehearsal marks in this TeleOp (match 2). */
    private void record() {
        if (recorder == null) return;
        try {
            fillSample();
            recorder.loop(runTimer.seconds(), System.currentTimeMillis(), false, false, sample);
            telemetry.addData("Recording", recorder.recordingStatus()
                    + (recordingProblem.isEmpty() ? "" : " (" + recordingProblem + ")"));
        } catch (RuntimeException ex) {
            // Never let recording stop the robot. Show the problem and stop recording.
            recordingProblem = ex.toString();
            recorder = null;
            telemetry.addData("Recording", "OFF: " + recordingProblem);
        }
    }

    /* Read the Pinpoint once per loop (route directions) and how fast the heading is changing. */
    private void updatePose() {
        if (odometry == null) { routeX = routeY = routeH = Double.NaN; return; }
        try {
            odometry.update();
            Pose pose = odometry.pose();
            double[] route = PollenFacing.routePose(pose.x(), pose.y(), pose.heading());
            double now = runTimer.seconds();
            if (Double.isFinite(previousH) && now > previousHAt)
                headingRate = PollenOdometryDrive.wrap(route[2] - previousH) / (now - previousHAt);
            previousH = route[2]; previousHAt = now;
            routeX = route[0]; routeY = route[1]; routeH = route[2];
        } catch (RuntimeException ex) {
            odometry = null;
            odometryProblem = "read failed: " + ex;
            routeX = routeY = routeH = Double.NaN;
        }
    }

    /* Keep the newest camera frame and when it arrived (a frame is new when its stamp changes). */
    private void readCamera() {
        if (camera == null) return;
        try {
            IsHiveReadyForShot.Frame f = camera.read();
            if (f != null && (lastFrame == null || f.stamp != lastFrameStamp)) {
                lastFrame = f; lastFrameStamp = f.stamp; lastFrameAt = runTimer.seconds();
            }
        } catch (RuntimeException ex) { cameraProblem = ex.toString(); }
    }

    private IsHiveReadyForShot.Frame freshFrame() {
        return runTimer.seconds() - lastFrameAt <= PollenHiveAim.MAX_FRAME_AGE_S ? lastFrame : null;
    }

    private String aimScreen = "";

    /* Circle: start (or cancel) an aim; while aiming, the turn comes from PollenHiveAim. */
    private void aimLoop() {
        double now = runTimer.seconds();
        if (aimCalibration()) {
            if (gamepad1.dpad_left && !aimLeftWas) aimSource = PollenHiveAim.Source.CAMERA;
            if (gamepad1.dpad_right && !aimRightWas) aimSource = PollenHiveAim.Source.ODOMETRY;
        }
        aimLeftWas = gamepad1.dpad_left;
        aimRightWas = gamepad1.dpad_right;
        boolean circle = gamepad1.circle;
        if (circle && !circleWasPressed) {
            // Circle while aiming = aim again from here (v37; v36 cancelled, and drivers pressed
            // again while the slow finish looked stuck). The right stick still cancels.
            if (aim.active()) logAim(aim.cancel(now, routeH, "restarted_by_circle"));
            String event = aim.start(now, aimSource, routeH, routeX, routeY, aimCalibration(),
                    freshFrame(), now - lastFrameAt, mount);
            logAim(event);
            if (aim.active()) gamepad1.rumble(150); else gamepad1.rumbleBlips(3);
        }
        circleWasPressed = circle;
        aimTurn = 0;
        if (!aim.active()) return;
        if (Math.abs(gamepad1.right_stick_x) > 0.25 || !Double.isFinite(routeH)) {
            logAim(aim.cancel(now, routeH, Double.isFinite(routeH) ? "driver_turned" : "no_heading"));
            return;
        }
        PollenHiveAim.Output o = aim.tick(now, routeH, headingRate, freshFrame(), now - lastFrameAt, mount);
        aimTurn = o.turn;
        aimScreen = o.screen;
        logAim(o.event);
        if (!o.active) gamepad1.rumbleBlips(1);
    }

    /* Write an aim event to the recording and the robot log. */
    private void logAim(String event) {
        if (event == null) return;
        RobotLog.ii("PollenAim", "%s", event);
        if (recorder == null) return;
        try { fillSample(); recorder.event(runTimer.seconds(), System.currentTimeMillis(), event, sample); }
        catch (RuntimeException ignored) { }
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // INIT drive-power setting: 100% leaves the normal stick response unchanged.
        leftFrontPower *= tuning.driveScale();
        rightFrontPower *= tuning.driveScale();
        leftBackPower *= tuning.driveScale();
        rightBackPower *= tuning.driveScale();

        /*
         * Send calculated power to wheels
         */
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    void launch() {
        /*
         * Calling gamepad1.right_bumper returns a boolean which will be true if the bumper is
         * held down, and false if it is not. Notably, this will continue to be true for every
         * cycle of our code that the driver holds down that bumper.
         * The first step of our launch() function is checking to see if the user is currently
         * holding down the right gamepad. If they are, then we want to start spinning up the launcher.
         * Otherwise, we start spinning the launcher down.
         */
        /*
         * L1 clears a jam (Square before v39): the launcher runs backwards at 300 RPM while L1 is
         * held. L1 wins over R1, and nothing feeds while reversing, so no ball is pushed into a wheel
         * turning the wrong way. Let go of L1 and R1 works as normal (with a fresh spin-up).
         */
        // Raise the drainer before reversing: the intake must pull inward while draining.
        reversing = gamepad1.left_bumper && !drainerDown;
        if (reversing) {
            launching = false;
            launcher.setVelocity(-LAUNCHER_REVERSE_VELOCITY);
        } else if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            if (!launching) launchTimer.reset(); // R1 just pressed: start the spin-up clock
            launching = true;
            launcher.setVelocity(tuning.launcherVelocity());
        } else {
            launching = false;
            launcher.setVelocity(0);
        }

        /*
         * When do we start feeding balls into the launcher? Whichever happens FIRST:
         *   1. the encoder says the launcher is up to speed (96%: about 1133 ticks/s for an 1180 target), or
         *   2. R1 has been held for the spin-up time (3 s), in case the encoder reading is wrong.
         * Once feeding starts, keep feeding until R1 is released. Each ball slows the wheel a
         * little, and we don't want that dip to switch the agitator off in the middle of a volley.
         */
        boolean upToSpeed = tuning.launcherIsUpToSpeed(launcherMeasuredVelocity);
        boolean spinUpTimeIsOver = tuning.spinUpTimeIsOver(launchTimer.seconds());
        if (!launching) {
            feeding = false;
        } else if (!feeding && (upToSpeed || spinUpTimeIsOver)) {
            feeding = true;
            feedStartedAt = launchTimer.seconds();
            feedStartedBy = upToSpeed ? "up to speed" : "spin-up timer";
        }
    }
}
