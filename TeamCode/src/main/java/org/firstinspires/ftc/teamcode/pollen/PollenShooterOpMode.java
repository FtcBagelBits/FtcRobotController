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

// Hardware conventions and launcher settings adapted from the goBILDA StarterBot sample.

package org.firstinspires.ftc.teamcode.pollen;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import static org.firstinspires.ftc.teamcode.pollen.PollenShootTuning.Field.*;

/** Shared hardware lifecycle for the two stationary shooting routines, timed or odometry. */
public abstract class PollenShooterOpMode extends OpMode {
    // Each of the four modes remembers its own settings: odometry defaults differ from timed ones.
    private static PollenShootTuning.Snapshot shootRemembered, supportRemembered,
            odoShootRemembered, odoSupportRemembered;
    protected abstract boolean supportingShooter();
    /** Odometry OpModes drive each leg to a measured end point instead of by time. */
    protected boolean odometry() { return false; }
    private PinpointLocalizer localizer;
    private GoBildaPinpointDriver pinpoint;
    private String pinpointStatus = "";
    /** Writes this run to the robot log (robotControllerLog.txt); see PollenAutoLog. */
    private PollenAutoLog log;
    private final DcMotor[] drive = new DcMotor[4];
    private DcMotorEx launcher;
    private DcMotor intake, agitator;
    private final ElapsedTime clock = new ElapsedTime();
    private PollenShootTuning tuning;
    private PollenSequence sequence;
    private String blocked, fallback;
    private boolean shooterAvailable;

    @Override public final void init() {
        blocked = fallback = null;
        sequence = null;
        shooterAvailable = true;
        log = new PollenAutoLog(modeName(), line -> RobotLog.ii(PollenAutoLog.TAG, "%s", line));
        tuning = new PollenShootTuning(remembered(), supportingShooter(), odometry());
        try {
            mapDrive();
            // Odometry first: the Pinpoint recalibrates its IMU now, while the robot is still.
            if (odometry()) {
                localizer = PollenOdometryHardware.open(hardwareMap);
                pinpoint = PollenOdometryHardware.device(hardwareMap);
            }
            try {
                mapShooter();
            } catch (RuntimeException ex) {
                if (!zeroShooter()) throw new IllegalStateException("Cannot confirm shooter stop", ex);
                shooterAvailable = false;
                fallback = "NO SHOT: " + ex + "; selected departure with shooting omitted";
            }
            // Validate the selected route before reporting ready. Over-budget remembered settings
            // are not an error here: INIT shows them and START refuses until they are fixed.
            if (tuning.budgetOk()) tuning.sequence(shooterAvailable);
        } catch (RuntimeException ex) {
            blocked = ex.toString();
            zeroHardware();
        }
        init_loop();
    }

    private void mapDrive() {
        for (int i = 0; i < drive.length; i++) {
            drive[i] = hardwareMap.get(DcMotor.class, PollenConfig.DRIVE_NAMES[i]);
            drive[i].setPower(0);
            drive[i].setDirection(PollenConfig.DRIVE_REVERSED[i]
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            drive[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void mapShooter() {
        // Team's BioBuzzStarterbotOpMode / LauncherTestOpMode mapping and goBILDA PIDF.
        launcher = hardwareMap.get(DcMotorEx.class, "pollenLauncher");
        launcher.setPower(0);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setPower(0);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        agitator = hardwareMap.get(DcMotor.class, "agitator");
        agitator.setPower(0);
        agitator.setDirection(DcMotorSimple.Direction.REVERSE);
        agitator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        agitator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override public final void init_loop() {
        if (gamepad1 != null) tuning.update(gamepad1.dpad_up, gamepad1.dpad_down,
                gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.triangle);
        remember(tuning.snapshot());
        log.initRoute(tuning.route().name());
        if (localizer != null) {
            localizer.update();
            readPinpointStatus();
        }
        showSettings();
        telemetry.addData("Controls", "D-pad UP/DOWN selects; LEFT/RIGHT edits; tap then release");
        telemetry.addData("Triangle", supportingShooter() ? "Restore defaults (flush route to the outer slot)"
                : "Restore defaults: shoot, then 3 legs to the parking slot");
        telemetry.addData("Circle / Square", "Do nothing");
        telemetry.addData("Tune", "Launcher speed, feed, power and each leg's cm/angle are rows above");
        telemetry.addData("Status", blocked != null ? "BLOCKED: " + blocked
                : !tuning.budgetOk() ? "NOT READY - over time budget; START will not move the robot"
                : PollenOdometryDrive.pinpointFault(pinpointStatus)
                ? "PINPOINT FAULT - odometry will not work: use the timed Pollen mode"
                : fallback != null ? "READY FOR DEPARTURE ONLY - SHOOTER FAILED" : "READY - no movement until START");
        telemetry.addData("Memory", "Settings kept until app restart; start delay resets every INIT");
        telemetry.update();
    }

    private PollenShootTuning.Snapshot remembered() {
        if (odometry()) return supportingShooter() ? odoSupportRemembered : odoShootRemembered;
        return supportingShooter() ? supportRemembered : shootRemembered;
    }

    private void remember(PollenShootTuning.Snapshot snapshot) {
        if (odometry()) {
            if (supportingShooter()) odoSupportRemembered = snapshot; else odoShootRemembered = snapshot;
        } else if (supportingShooter()) supportRemembered = snapshot; else shootRemembered = snapshot;
    }

    private void showSettings() {
        // First line on purpose: which route will run is the thing to check before every match.
        telemetry.addData("ROUTE", tuning.route().label);
        telemetry.addData("Build", PollenConfig.BUILD);
        telemetry.addData("Mode", (odometry() ? "ODOMETRY " : "")
                + (supportingShooter() ? "Supporting Shooter - real motors on START" : "Shoot + Park - real motors on START"));
        if (localizer != null) {
            PollenOdometryHardware.showPose(telemetry, localizer.pose());
            telemetry.addData("Pinpoint status", PollenOdometryDrive.pinpointFault(pinpointStatus)
                    ? pinpointStatus + "  <-- FAULT: check the pod cables" : pinpointStatus);
        }
        telemetry.addData("Selections", tuning.locked() ? "LOCKED at START" : "INIT editing");
        telemetry.addData("Start pose", supportingShooter() ? "End wall, FLUSH + SQUARE, centered between flower and midline"
                : "End wall, flush + square, aligned with initially UP cell");
        for (PollenShootTuning.Field f : PollenShootTuning.Field.values()) {
            telemetry.addData((f == tuning.selected() ? "> " : "  ") + tuning.label(f), "%.2f", tuning.value(f));
        }
        if (odometry()) {
            showOdometryRoute();
        } else for (int leg = 1; leg <= PollenShootTuning.LEGS; leg++) {
            PollenDriveCalibration.Move m = tuning.legMove(leg);
            telemetry.addData(tuning.legName(leg), tuning.legCm(leg) <= 0 ? "skipped (0 cm)"
                    : String.format(java.util.Locale.US, "%.1f cm @ %+.1f deg -> %.2f s; command %+.1f deg; NO TURN",
                    tuning.legCm(leg), tuning.legAngle(leg), m.seconds, m.commandAngleDegrees));
        }
        telemetry.addData(odometry() ? "Planned" : "Worst case", "%.2f s; must be < %.0f s (hard stop at %.0f s)",
                tuning.worstCaseSeconds(), PollenShootTuning.DEADLINE - 1, PollenShootTuning.DEADLINE);
        if (!odometry()) {
            telemetry.addData("Calibration", tuning.profile());
            telemetry.addData("Angles", "+left / -right of start heading; MAT TRIAL converts desired path to command");
            if (tuning.profile() == PollenDriveCalibration.Profile.MAT_TRIAL)
                telemetry.addData("MAT TRIAL", "UNVERIFIED: assumes unchanged 129 test settings; repeat short tests");
        }
        if (supportingShooter()) {
            telemetry.addData("Support placement", "Center %.1f cm toward midline from hive center; gaps %.1f cm flower / %.1f cm midline",
                    PollenFieldGeometry.supportOffsetFromHiveCm(), PollenFieldGeometry.supportFlowerGapCm(),
                    PollenFieldGeometry.supportMidlineGapCm());
            telemetry.addData("FLOWER CLEARANCE", "Tight nominal start; check actual flower, midline and full footprint");
            telemetry.addData("Departure", "about %.1f s; without a shooter C still waits out the slot",
                    tuning.feedStartSeconds() + tuning.value(FEED_SECONDS));
            if (tuning.legCm(1) == 0 && tuning.legCm(2) == 0 && tuning.legCm(3) == 0)
                telemetry.addData("NO PARKING MOVE", "All legs are 0 cm: enter measured travel before match");
        }
        if (tuning.value(FEED_SECONDS) > 0) telemetry.addData("Shot timing",
                "launcher on at %.1f s, feed %.1f-%.1f s (fixed times; speed is not checked)",
                tuning.feedStartSeconds() - tuning.value(SPINUP), tuning.feedStartSeconds(),
                tuning.feedStartSeconds() + tuning.value(FEED_SECONDS));
        if (!tuning.budgetOk()) telemetry.addData("OVER TIME BUDGET",
                "%.2f s planned, must be < %.0f s: lower delay or feed, raise power, or shorten legs",
                tuning.worstCaseSeconds(), PollenShootTuning.DEADLINE - 1);
        if (fallback != null) telemetry.addData("SHOOTER INIT FAILED", fallback);
        if (!tuning.notice().isEmpty()) telemetry.addData("Settings notice", tuning.notice());
    }

    /** Odometry mode: where each leg ends, measured from the zero point at the start. */
    private void showOdometryRoute() {
        telemetry.addData("First", tuning.wallSquare()
                ? "back into the end wall %.2f s at %.2f power, then zero odometry (inside the start delay)"
                : "zero odometry where the robot stands (wall square OFF)",
                PollenOdometryDrive.SQUARE_SECONDS, PollenOdometryDrive.SQUARE_POWER);
        for (int leg = 1; leg <= PollenShootTuning.LEGS; leg++) {
            double[] end = tuning.legEndInches(leg);
            telemetry.addData(tuning.legName(leg), tuning.legCm(leg) <= 0 ? "skipped (0 cm)"
                    : String.format(java.util.Locale.US, "%.1f cm @ %+.1f deg -> ends %.1f in fwd, %.1f in left; about %.1f s",
                    tuning.legCm(leg), tuning.legAngle(leg), end[0], end[1], tuning.odometryLegSeconds(leg)));
        }
        telemetry.addData("Angles", "+left / -right of the zeroed heading; distances are measured, not timed");
    }

    @Override public final void start() {
        tuning.lock();
        clock.reset();
        log.start(tuning.settingsLine(), "shooter=" + (shooterAvailable ? "ok" : "MISSING")
                + (odometry() ? " pinpoint=" + pinpointStatus : ""), this::batteryVolts);
        if (blocked == null && !tuning.budgetOk()) {
            blocked = "NOT STARTED: settings over time budget (fix in INIT, or Triangle for defaults)";
            telemetry.addData("Status", blocked);
            telemetry.update();
        }
        if (blocked != null) { log.notStarted(blocked); zeroHardware(); requestOpModeStop(); return; }
        try {
            sequence = tuning.sequence(shooterAvailable);
            sequence.start(0);
        } catch (RuntimeException ex) {
            fail(ex);
        }
    }

    /** Every loop does the same four things in the same order: read, decide, act, report.
     * Nothing in here waits: it returns, and the SDK calls it again a moment later. */
    @Override public final void loop() {
        if (blocked != null || sequence == null) return;
        try {
            // 1. READ the clock and each sensor once.
            double now = clock.seconds();
            double x = Double.NaN, y = Double.NaN, heading = Double.NaN; // stay NaN in timed modes
            if (localizer != null) {
                localizer.update();
                readPinpointStatus();
                Pose pose = localizer.pose();
                x = pose.x();
                y = pose.y();
                heading = pose.heading();
            }
            double launcherMeasured = shooterAvailable ? launcher.getVelocity() : 0; // screen and log only; never used for timing

            // 2. DECIDE: the sequence says what every motor should do right now.
            PollenSequence.Output out = sequence.tick(now, x, y, heading);
            double[] wheels = PollenSequence.wheelPowers(out); // frontLeft, frontRight, backLeft, backRight
            double intakePower = out.feederPower * tuning.value(FEED_POWER);
            double agitatorPower = out.feederPower * tuning.value(AGITATOR_POWER);

            // 3. ACT: write each output once. The shooter comes before the wheels, so the
            // shooting mechanisms stop BEFORE the first parking drive command.
            if (out.zeroOdometry && localizer != null) localizer.setPose(Pose.zero());
            if (shooterAvailable) {
                launcher.setVelocity(out.launcherVelocity);
                intake.setPower(intakePower);
                agitator.setPower(agitatorPower);
            }
            for (int i = 0; i < drive.length; i++) drive[i].setPower(wheels[i]);

            // 4. REPORT to the Driver Hub screen and the robot log.
            showSettings();
            telemetry.addData("Phase", out.phase);
            telemetry.addData("Elapsed", "%.2f s", now);
            telemetry.addData("Launcher target / measured", "%.0f / %.0f ticks/s", out.launcherVelocity, launcherMeasured);
            telemetry.addData("Intake / agitator command", "%.2f / %.2f", intakePower, agitatorPower);
            telemetry.addData("Commanded feed time", "%.2f s", out.activeFeedSeconds);
            telemetry.addData("FL FR BL BR", "%.3f %.3f %.3f %.3f", wheels[0], wheels[1], wheels[2], wheels[3]);
            if (!Double.isNaN(out.inchesToGo)) telemetry.addData("Inches to go", "%.1f", out.inchesToGo);
            if (!sequence.warning().isEmpty()) telemetry.addData("Warning", sequence.warning());
            telemetry.update();
            log.loop(now, out, sequence.warning(), x, y, heading, wheels, launcherMeasured, this::batteryVolts);
        } catch (RuntimeException ex) {
            fail(ex);
        }
    }

    private void fail(RuntimeException ex) {
        RobotLog.ee(PollenAutoLog.TAG, ex, "ERROR - all outputs stopped");
        blocked = ex.toString();
        if (sequence != null) sequence.stop("HARDWARE ERROR");
        zeroHardware();
        telemetry.addData("ERROR - ALL OUTPUTS STOPPED", blocked);
        telemetry.update();
        requestOpModeStop();
    }

    @Override public final void stop() {
        if (tuning != null) tuning.lock();
        if (sequence != null) sequence.stop("STOPPED");
        zeroHardware();
        if (log != null) log.stop(clock.seconds());
    }

    /** Name as shown on the Driver Hub, for the log. */
    private String modeName() {
        return (odometry() ? "Odo " : "Pollen ") + (supportingShooter() ? "Supporting Shooter" : "ROBOT Shoot + Park");
    }

    /** Pinpoint status for the screen and the log (refreshed by localizer.update()). */
    private void readPinpointStatus() {
        pinpointStatus = PollenOdometryHardware.status(pinpoint);
        log.status(pinpointStatus);
    }

    /** Lowest reading of the hub voltage sensors, for the log. NaN if none. */
    private double batteryVolts() {
        double volts = Double.NaN;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0 && (Double.isNaN(volts) || v < volts)) volts = v;
        }
        return volts;
    }

    private void zeroHardware() {
        zeroShooter();
        for (DcMotor motor : drive) zero(motor);
    }

    private boolean zeroShooter() {
        boolean ok = zero(launcher);
        ok &= zero(intake);
        ok &= zero(agitator);
        return ok;
    }

    private boolean zero(DcMotor motor) {
        if (motor == null) return true;
        try { motor.setPower(0); return true; }
        catch (RuntimeException ex) { telemetry.addData("Stop motor error", ex.toString()); return false; }
    }
}
