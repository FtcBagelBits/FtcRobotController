package org.firstinspires.ftc.teamcode;

import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pollen.PollenConfig;

import java.util.Locale;

/**
 * Baseline check for the Pinpoint odometry computer, before Pedro Pathing is trusted.
 * The step-by-step procedure and pass/fail numbers are in docs/ODOMETRY_BASELINE_TEST.md.
 *
 * It talks to the Pinpoint directly, not through Pedro, so it can show things Pedro hides:
 * the device status (pods plugged in?) and the raw count from each pod. It sets the Pinpoint up
 * with the SAME numbers Pedro uses (pedroPathing/Constants.localizerConfig), so whatever passes
 * here is what Pedro will see.
 *
 * Nothing moves until START, and after START only the sticks move the robot. Every hand-push
 * test can be done during INIT.
 *
 * Pinpoint directions (also Pedro's): X = forward, Y = LEFT, heading counterclockwise-positive.
 *
 * Controls (INIT or after START):
 *   Cross    - zero the pose here (x = 0, y = 0, heading = 0) and restart the spin numbers.
 *   Triangle - recalibrate the IMU and zero. The robot must be completely still.
 * After START:
 *   Left stick drives and strafes, right stick turns, same as PollenBot TeleOp, at reduced power.
 */
@TeleOp(name = "Odometry Check", group = "Pedro")
public class OdometryCheck extends OpMode {
    /** Stick scaling after START, so the robot is slow enough to watch and measure. */
    private static final double DRIVE_SCALE = 0.4;

    /** goBILDA's published pod resolutions, ticks per millimetre (only used for display). */
    private static final double FOUR_BAR_TICKS_PER_MM = 19.89436789;
    private static final double SWINGARM_TICKS_PER_MM = 13.26291192;

    private final PinpointConfig cfg = Constants.localizerConfig;
    private GoBildaPinpointDriver pinpoint;
    private final DcMotor[] drive = new DcMotor[4];
    private boolean driveReady;
    private String problem = "";

    // Values captured when Cross is pressed, so everything below reads "since zero".
    private int rawXAtZero, rawYAtZero;
    private double lastHeadingDeg, turnedDeg, maxDistanceIn;
    private boolean previousCross, previousTriangle;
    private final ElapsedTime loopTimer = new ElapsedTime();
    /** Time since the last zero. The Pinpoint may report the old pose (or be calibrating) for a
     * moment after a zero, so the "since zero" baselines are re-captured until this settles. */
    private final ElapsedTime sinceZero = new ElapsedTime();
    private static final double ZERO_SETTLE_SECONDS = 0.35;

    @Override
    public void init() {
        pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, cfg.name.get());
        if (pinpoint == null) {
            problem = "No goBILDA Pinpoint named \"" + cfg.name.get() + "\" in the active "
                    + "configuration. Check the name, and that its type is goBILDA Pinpoint "
                    + "Odometry Computer (a device with the right name but another type is not found).";
        } else {
            // Same order and values as Pedro's PinpointLocalizer constructor.
            pinpoint.setOffsets(cfg.xPodOffset.get(), cfg.yPodOffset.get(), cfg.offsetUnits.get());
            if (cfg.ticksPerUnit.get().isPresent()) {
                pinpoint.setEncoderResolution(cfg.ticksPerUnit.get().getAsDouble(),
                        cfg.encoderResolutionUnit.get());
            } else {
                pinpoint.setEncoderResolution(cfg.podType.get());
            }
            pinpoint.setEncoderDirections(cfg.xPodDirection.get(), cfg.yPodDirection.get());
            pinpoint.resetPosAndIMU();
            sinceZero.reset();
        }

        // Drive motors are optional: the hand-push tests do not need them.
        driveReady = true;
        for (int i = 0; i < drive.length; i++) {
            drive[i] = hardwareMap.tryGet(DcMotor.class, PollenConfig.DRIVE_NAMES[i]);
            if (drive[i] == null) {
                driveReady = false;
                continue;
            }
            drive[i].setPower(0);
            drive[i].setDirection(PollenConfig.DRIVE_REVERSED[i]
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            drive[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        telemetry.addLine("Calibrating - do not touch the robot for 1 second.");
    }

    @Override
    public void init_loop() {
        readAndShow("INIT - hand-push tests are safe now; motors are off");
    }

    @Override
    public void start() {
        zero();
    }

    @Override
    public void loop() {
        if (driveReady) {
            // Same mixing as PollenBot TeleOp and PollenSequence.wheelPowers:
            // forward +, strafe + = right, turn + = clockwise. Order FL, FR, BL, BR.
            double forward = -gamepad1.left_stick_y * DRIVE_SCALE;
            double strafe = gamepad1.left_stick_x * DRIVE_SCALE;
            double turn = gamepad1.right_stick_x * DRIVE_SCALE;
            double[] powers = {forward + strafe + turn, forward - strafe - turn,
                    forward - strafe + turn, forward + strafe - turn};
            double max = 1;
            for (double p : powers) max = Math.max(max, Math.abs(p));
            for (int i = 0; i < drive.length; i++) drive[i].setPower(powers[i] / max);
        }
        readAndShow(driveReady ? "RUNNING - sticks drive at " + (int) (DRIVE_SCALE * 100) + "%"
                : "RUNNING - drive motors missing, sticks do nothing");
    }

    @Override
    public void stop() {
        for (DcMotor motor : drive) if (motor != null) motor.setPower(0);
    }

    /** Pose and spin numbers restart from here. Does not recalibrate the IMU. */
    private void zero() {
        if (pinpoint == null) return;
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        sinceZero.reset();
    }

    private void readAndShow(String mode) {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        telemetry.addData("Mode", mode);
        if (pinpoint == null) {
            telemetry.addData("PROBLEM", problem);
            telemetry.update();
            return;
        }

        // Buttons act once per press.
        if (gamepad1.triangle && !previousTriangle) {
            pinpoint.resetPosAndIMU();
            zero();
        } else if (gamepad1.cross && !previousCross) {
            zero();
        }
        previousTriangle = gamepad1.triangle;
        previousCross = gamepad1.cross;

        pinpoint.update();
        double x = pinpoint.getPosX(DistanceUnit.INCH);
        double y = pinpoint.getPosY(DistanceUnit.INCH);
        double headingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
        if (sinceZero.seconds() < ZERO_SETTLE_SECONDS) {
            rawXAtZero = pinpoint.getEncoderX();
            rawYAtZero = pinpoint.getEncoderY();
            lastHeadingDeg = headingDeg;
            turnedDeg = wrapDegrees(headingDeg);
            maxDistanceIn = 0;
        }
        turnedDeg += wrapDegrees(headingDeg - lastHeadingDeg);
        lastHeadingDeg = headingDeg;
        double distance = Math.hypot(x, y);
        maxDistanceIn = Math.max(maxDistanceIn, distance);
        int rawX = pinpoint.getEncoderX() - rawXAtZero;
        int rawY = pinpoint.getEncoderY() - rawYAtZero;

        telemetry.addData("Pinpoint status", "%s   (%.0f Hz, OpMode loop %.0f ms)",
                pinpoint.getDeviceStatus(), pinpoint.getFrequency(), loopMs);
        telemetry.addLine("--- Since zero (Cross) ---");
        telemetry.addData("X forward (in)", "%7.2f", x);
        telemetry.addData("Y left (in)", "%7.2f", y);
        telemetry.addData("Heading (deg)", "%7.2f   total turned %7.1f", headingDeg, turnedDeg);
        telemetry.addData("Raw X pod", "%7d ticks = %6.2f in", rawX, rawX / ticksPerInch());
        telemetry.addData("Raw Y pod", "%7d ticks = %6.2f in", rawY, rawY / ticksPerInch());
        telemetry.addData("From zero now / max (in)", "%5.2f / %5.2f", distance, maxDistanceIn);
        telemetry.addData("Offset suggestion", offsetSuggestion(x, y));
        telemetry.addLine("--- Settings in use ---");
        telemetry.addData("Pinpoint (Constants)", "%s; offsets x %.1f, y %.1f %s; dirs X %s, Y %s",
                cfg.ticksPerUnit.get().isPresent() ? "custom pods" : cfg.podType.get(),
                cfg.xPodOffset.get(), cfg.yPodOffset.get(), cfg.offsetUnits.get(),
                cfg.xPodDirection.get(), cfg.yPodDirection.get());
        telemetry.addData("Pedro drive vs PollenConfig", pedroDriveCheck());
        telemetry.addData("Cross / Triangle", "zero here / recalibrate IMU + zero (robot still!)");
        telemetry.update();
    }

    /**
     * After an in-place turn, any movement of the tracked point means the offsets put the
     * tracked point somewhere other than the center the robot turned about. Turning by angle t
     * about the true center moves the tracked point by d = (R(t) - I) e, where e is how far the
     * tracked point sits from the true center (forward, left) in the robot's starting frame.
     * Solving for e and moving the offsets by e makes the tracked point the turning center.
     * Only well-conditioned between 90 and 270 degrees of turn; 180 is best.
     */
    private String offsetSuggestion(double dxIn, double dyIn) {
        double turned = Math.abs(turnedDeg);
        if (turned < 90 || turned > 270) {
            return "zero, then turn in place 180 deg (now " + Math.round(turnedDeg) + ")";
        }
        double unitPerInch = cfg.offsetUnits.get().fromInches(1);
        double[] suggested = suggestOffsets(cfg.xPodOffset.get(), cfg.yPodOffset.get(),
                unitPerInch, dxIn, dyIn, Math.toRadians(turnedDeg));
        return String.format(Locale.US, "xPodOffset %.1f, yPodOffset %.1f %s (after %.0f deg)",
                suggested[0], suggested[1], cfg.offsetUnits.get(), turnedDeg);
    }

    /**
     * Suggested (xPodOffset, yPodOffset) in the config's units. Pinpoint convention: xPodOffset is
     * how far LEFT of the tracked point the forward (X) pod is; yPodOffset is how far FORWARD of
     * it the strafe (Y) pod is. Static and hardware-free so it can be checked off the robot.
     */
    static double[] suggestOffsets(double xPodOffset, double yPodOffset, double unitPerInch,
                                   double dxIn, double dyIn, double turnedRad) {
        double c = Math.cos(turnedRad) - 1, s = Math.sin(turnedRad);
        double det = c * c + s * s;
        // e = (R - I)^-1 d, with R - I = [[c, -s], [s, c]].
        double forwardErrorIn = (c * dxIn + s * dyIn) / det;
        double leftErrorIn = (-s * dxIn + c * dyIn) / det;
        return new double[] {
                xPodOffset + leftErrorIn * unitPerInch,
                yPodOffset + forwardErrorIn * unitPerInch
        };
    }

    private double ticksPerInch() {
        if (cfg.ticksPerUnit.get().isPresent()) {
            return cfg.ticksPerUnit.get().getAsDouble() / cfg.encoderResolutionUnit.get().toInches(1);
        }
        boolean fourBar = cfg.podType.get() == GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        return (fourBar ? FOUR_BAR_TICKS_PER_MM : SWINGARM_TICKS_PER_MM) * 25.4;
    }

    /** Pedro drives with its own copy of the motor names and directions; they must match the autos. */
    private static String pedroDriveCheck() {
        MecanumConfig m = Constants.drivetrainConfig;
        String[] names = {m.frontLeftName.get(), m.frontRightName.get(),
                m.backLeftName.get(), m.backRightName.get()};
        DcMotorSimple.Direction[] dirs = {m.frontLeftDirection.get(), m.frontRightDirection.get(),
                m.backLeftDirection.get(), m.backRightDirection.get()};
        StringBuilder mismatch = new StringBuilder();
        for (int i = 0; i < 4; i++) {
            DcMotorSimple.Direction pollen = PollenConfig.DRIVE_REVERSED[i]
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
            if (!names[i].equals(PollenConfig.DRIVE_NAMES[i]) || dirs[i] != pollen) {
                mismatch.append(PollenConfig.DRIVE_NAMES[i]).append(' ');
            }
        }
        return mismatch.length() == 0 ? "match" : "MISMATCH: " + mismatch;
    }

    private static double wrapDegrees(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees <= -180) degrees += 360;
        return degrees;
    }
}
