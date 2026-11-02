package org.firstinspires.ftc.teamcode.pollen;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Option A with odometry: after the start delay, zero the odometry, drive straight forward the
 * selected distance (measured, not timed), then stop. Same INIT buttons as Pollen ROBOT Park,
 * except LEFT/RIGHT changes centimetres instead of seconds. */
@Autonomous(name = "Odo ROBOT Park", group = "Pollen Odometry", preselectTeleOp = "PollenBot")
public final class PollenOdoPark extends OpMode {
    private static PollenOdoParkTuning.Snapshot remembered;
    private final DcMotor[] drive = new DcMotor[4];
    private final ElapsedTime clock = new ElapsedTime();
    private PinpointLocalizer odometry;
    private PollenOdoParkTuning tuning;
    private PollenSequence sequence;
    private String blocked;

    @Override public void init() {
        blocked = null;
        tuning = null;
        sequence = null;
        try {
            for (int i = 0; i < drive.length; i++) {
                drive[i] = hardwareMap.get(DcMotor.class, PollenConfig.DRIVE_NAMES[i]);
                drive[i].setPower(0);
                drive[i].setDirection(PollenConfig.DRIVE_REVERSED[i]
                        ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
                drive[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drive[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            odometry = PollenOdometryHardware.open(hardwareMap);
            tuning = new PollenOdoParkTuning(PollenConfig.PARK_FORWARD_POWER, PollenConfig.START_DELAY_SECONDS,
                    PollenConfig.TOTAL_TIME_LIMIT, PollenConfig.PARK_HOLD_SECONDS, remembered);
            sequence = tuning.sequence();
        } catch (RuntimeException ex) {
            blocked = ex.toString();
            zeroHardware();
        }
        init_loop();
    }

    @Override public void init_loop() {
        if (tuning != null && gamepad1 != null) {
            tuning.update(gamepad1.dpad_up, gamepad1.dpad_down,
                    gamepad1.dpad_left, gamepad1.dpad_right,
                    gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.triangle);
            remembered = tuning.snapshot();
        }
        if (odometry != null) odometry.update();
        showSettings();
        telemetry.addData("D-pad", "UP/DOWN delay +/-0.5s; RIGHT/LEFT distance +/-5 cm");
        telemetry.addData("L1 / R1", "Decrease / increase power by 0.05");
        telemetry.addData("Triangle", "Reset power, distance, delay to code defaults");
        telemetry.addData("Placement", "Set starting position and angle by hand before START");
        telemetry.addData("Status", blocked == null ? "READY - waiting for START" : "BLOCKED: " + blocked);
        telemetry.update();
    }

    private void showSettings() {
        telemetry.addData("Build", PollenConfig.BUILD);
        telemetry.addData("Mode", "ODOMETRY ROBOT Park - real motors on START");
        if (odometry != null) PollenOdometryHardware.showPose(telemetry, odometry.pose());
        if (tuning == null) return;
        telemetry.addData("Distance / max power", "%.0f cm (%.1f in) straight ahead / %.2f",
                tuning.cm(), tuning.cm() / PollenOdometryDrive.CM_PER_INCH, tuning.power());
        telemetry.addData("Start delay / maximum", "%.1f / %.1f seconds",
                tuning.delaySeconds(), tuning.maximumDelaySeconds());
        telemetry.addData("Planned", "%.1f s (hard stop at %.0f s)", tuning.plannedSeconds(), PollenConfig.TOTAL_TIME_LIMIT);
        telemetry.addData("Selection", tuning.locked() ? "LOCKED" : "Adjust on Gamepad 1 before START");
    }

    @Override public void start() {
        clock.reset();
        if (tuning != null) tuning.lock();
        if (blocked != null) { zeroHardware(); requestOpModeStop(); return; }
        try {
            sequence = tuning.sequence();
            sequence.start(0);
        } catch (RuntimeException ex) { fail(ex); }
    }

    @Override public void loop() {
        if (blocked != null || sequence == null) return;
        try {
            double now = clock.seconds();
            odometry.update();
            Pose pose = odometry.pose();
            PollenSequence.Output out = sequence.tick(now, pose.x(), pose.y(), pose.heading());
            if (out.zeroOdometry) odometry.setPose(Pose.zero());
            double[] powers = PollenSequence.wheelPowers(out);
            for (int i = 0; i < drive.length; i++) drive[i].setPower(powers[i]);
            showSettings();
            telemetry.addData("Phase", out.phase);
            telemetry.addData("Elapsed seconds", "%.2f", now);
            if (!Double.isNaN(out.inchesToGo)) telemetry.addData("Inches to go", "%.1f", out.inchesToGo);
            telemetry.addData("FL FR BL BR", "%.2f %.2f %.2f %.2f",
                    powers[0], powers[1], powers[2], powers[3]);
            if (!sequence.warning().isEmpty()) telemetry.addData("Warning", sequence.warning());
            telemetry.update();
        } catch (RuntimeException ex) { fail(ex); }
    }

    private void fail(RuntimeException ex) {
        blocked = ex.toString();
        if (sequence != null) sequence.stop("HARDWARE ERROR");
        zeroHardware();
        telemetry.addData("ERROR - stopped", blocked);
        telemetry.update();
        requestOpModeStop();
    }

    @Override public void stop() {
        if (tuning != null) tuning.lock();
        if (sequence != null) sequence.stop("STOPPED");
        zeroHardware();
    }

    private void zeroHardware() {
        for (DcMotor motor : drive) {
            if (motor == null) continue;
            try { motor.setPower(0); }
            catch (RuntimeException ex) { telemetry.addData("Stop motor error", ex.toString()); }
        }
    }
}
