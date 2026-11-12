package org.firstinspires.ftc.teamcode.pollen;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.Locale;
import java.io.File;

/** Hardware adapter for the experimental hive autos. Legacy match autos are unchanged.
 * Normal use: place the robot, pick the auto, INIT, START. Every number is in the INIT menu with
 * its default (PollenHiveSettings); anything that differs from the default is marked CHANGED.
 * D-pad up/down = row, LB/RB = jump 5 rows, left/right = change, Triangle = this row to default,
 * Circle = route rows to default, preserving execution and camera, Square = save the changes on the Hub. */
public abstract class PollenHiveOpMode extends OpMode {
    protected abstract boolean secondary();
    protected PollenHiveAuto.Run run() { return PollenHiveAuto.Run.MATCH; }
    private int defaultExecution() { return run()==PollenHiveAuto.Run.MATCH?2:run()==PollenHiveAuto.Run.GARDEN_RETURN?1:0; }
    private File reportFile;
    private String reportError = "";
    private IsHiveReadyForShot initHive;
    private String initProfile = "";
    private final ElapsedTime clock = new ElapsedTime();
    private final DcMotor[] drive = new DcMotor[4];
    private DcMotorEx launcher;
    private DcMotor intake, agitator;
    private PinpointLocalizer localizer;
    private GoBildaPinpointDriver pinpoint;
    private PollenHiveRoute settings;
    private PollenHiveSettings.Values cameraSettings;
    private PollenHiveCamera camera;
    private String cameraProblem = "", loadMessage = "", message = "";
    private IsHiveReadyForShot hive;
    private PollenHiveAuto auto;
    private PollenHiveAuto.Output last;
    private String fault, logged = "";
    private int row;
    private boolean up, down, left, right, lb, rb, square, triangle, circle;
    private double displayAt = -1, hiveLogAt = -1;

    private String fileName() { return secondary() ? PollenHiveStore.SECONDARY : PollenHiveStore.PRIMARY; }
    private int rows() { return settings.v.length + cameraSettings.v.length; }
    private PollenHiveSettings.Values table(int r) { return r < settings.v.length ? settings.values : cameraSettings; }
    private int index(int r) { return r < settings.v.length ? r : r - settings.v.length; }

    @Override public final void init() {
        settings = new PollenHiveRoute(secondary());
        cameraSettings = new PollenHiveSettings.Values(PollenHiveSettings.CAMERA, false);
        loadMessage = "Auto: " + PollenHiveStore.load(fileName(), settings.values)
                + ". Camera: " + PollenHiveStore.load(PollenHiveStore.CAMERA, cameraSettings) + ".";
        settings.v[0] = defaultExecution();
        try {
            for (int i = 0; i < 4; i++) {
                drive[i] = hardwareMap.get(DcMotor.class, PollenConfig.DRIVE_NAMES[i]); drive[i].setPower(0);
                drive[i].setDirection(PollenConfig.DRIVE_REVERSED[i] ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
                drive[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drive[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            launcher = hardwareMap.get(DcMotorEx.class, "pollenLauncher"); launcher.setPower(0);
            launcher.setDirection(DcMotorSimple.Direction.REVERSE);
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
            intake = hardwareMap.get(DcMotor.class, "intake"); intake.setPower(0);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            agitator = hardwareMap.get(DcMotor.class, "agitator"); agitator.setPower(0);
            agitator.setDirection(DcMotorSimple.Direction.REVERSE);
            for (DcMotor m : new DcMotor[]{intake, agitator}) {
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            localizer = PollenOdometryHardware.open(hardwareMap);
            pinpoint = PollenOdometryHardware.device(hardwareMap);
        } catch (Exception ex) { fault = ex.toString(); zero(); }
        // A missing camera is not a fault: the auto still drives and parks, it just never shoots.
        try { camera = new PollenHiveCamera(hardwareMap, (int) cameraSettings.get("pipeline")); }
        catch (Exception ex) { camera = null; cameraProblem = ex.toString(); }
        init_loop();
    }

    @Override public final void init_loop() {
        menu();
        try { if (localizer != null) localizer.update(); } catch (Exception ex) { fault = ex.toString(); }
        try { if (camera != null && camera.pipeline() != (int) cameraSettings.get("pipeline")) camera.setPipeline((int) cameraSettings.get("pipeline")); }
        catch (Exception ex) { cameraProblem = ex.toString(); }
        String validation = "READY";
        try { settings.validate(); } catch (Exception ex) { validation = "BLOCKED: " + ex.getMessage(); }
        IsHiveReadyForShot.Mount mount = IsHiveReadyForShot.Mount.from(cameraSettings);
        int changed = settings.values.changedCount() + cameraSettings.changedCount();
        int unsaved = settings.values.unsavedCount() + cameraSettings.unsavedCount();
        telemetry.addData("BUILD", PollenConfig.BUILD);
        telemetry.addData("RUN", run() + " / " + PollenHiveSettings.format("execution", settings.v[0]));
        telemetry.addData("Motor limits", run()==PollenHiveAuto.Run.SHOT_ONLY?"NO DRIVE; LIVE enables launcher/feed":run()==PollenHiveAuto.Run.GARDEN_RETURN?"NO LAUNCHER/AGITATOR; LIVE adds collection intake":"Match auto: selected execution");
        telemetry.addData("Plan", run()==PollenHiveAuto.Run.SHOT_ONLY ? "Stationary volley, then DONE; no departure"
                : run()==PollenHiveAuto.Run.GARDEN_RETURN ? "Garden, return to S, settle, DONE; no shots or park"
                : secondary() ? "SECONDARY: wait for our cell UP, shoot, park"
                : "PRIMARY: shoot, garden (front intake), return, wait/shoot, park");
        telemetry.addData("SETTINGS", changed == 0 ? "all defaults" : "*** " + changed + " CHANGED FROM DEFAULT: "
                + join(settings.values.changedSummary(), cameraSettings.changedSummary()) + " ***");
        if (unsaved > 0) telemetry.addData("UNSAVED", unsaved + " edit(s): Square saves, Circle = route defaults");
        telemetry.addData("Hive cell behind robot", hiveView(mount));
        telemetry.addData("Camera calibration", mount.problem().isEmpty() ? String.format(Locale.US,
                "tilt %.1f deg, lens %.2f in", mount.tiltUpDeg, mount.lensHeightIn) : "NOT CALIBRATED - will not shoot");
        telemetry.addData("Estimated garden / park s", "%.1f / %.1f",
                PollenHiveRoute.planned(settings.garden()), PollenHiveRoute.planned(settings.park()));
        telemetry.addData("Odometry", PollenOdometryHardware.status(pinpoint));
        telemetry.addData("Status", fault == null ? validation : fault);
        for (int r = Math.max(0, Math.min(row - 3, rows() - 7)); r <= Math.min(rows() - 1, Math.max(row + 3, 6)); r++)
            telemetry.addLine((r == row ? "> " : "   ") + (r + 1) + "/" + rows() + "  "
                    + (r == settings.v.length ? "[camera] " : "") + table(r).line(index(r)));
        telemetry.addLine("D-pad row/value, LB/RB jump 5. Triangle: row to default. Circle: ROUTE defaults (keeps execution/camera). Square: save.");
        telemetry.addLine(message.isEmpty() ? loadMessage : message);
        telemetry.update();
    }

    private void menu() {
        int n = rows();
        if (gamepad1.dpad_up && !up) row = (row + n - 1) % n;
        if (gamepad1.dpad_down && !down) row = (row + 1) % n;
        if (gamepad1.left_bumper && !lb) row = Math.max(0, row - 5);
        if (gamepad1.right_bumper && !rb) row = Math.min(n - 1, row + 5);
        int delta = (gamepad1.dpad_right && !right ? 1 : 0) - (gamepad1.dpad_left && !left ? 1 : 0);
        if (delta != 0) table(row).edit(index(row), delta);
        if (gamepad1.triangle && !triangle) { table(row).reset(index(row)); if(row==0) settings.v[0]=defaultExecution(); message = "Row back to default (Square saves)"; }
        if (gamepad1.circle && !circle) { settings.values.resetExceptExecution(); message = "Route defaults restored; execution and camera kept (Square saves)"; }
        if (gamepad1.square && !square) {
            try {
                settings.validate();
                PollenHiveStore.save(fileName(), settings.values);
                PollenHiveStore.save(PollenHiveStore.CAMERA, cameraSettings);
                message = "SAVED on the Hub: " + nonEmpty(join(settings.values.changedSummary(), cameraSettings.changedSummary()), "all defaults");
                RobotLog.ii("PollenHive", "SAVED %s: %s | camera: %s", fileName(), settings.values.overrides(), cameraSettings.overrides());
            } catch (Exception ex) { message = "NOT SAVED: " + ex.getMessage(); }
        }
        up = gamepad1.dpad_up; down = gamepad1.dpad_down; left = gamepad1.dpad_left; right = gamepad1.dpad_right;
        lb = gamepad1.left_bumper; rb = gamepad1.right_bumper;
        square = gamepad1.square; triangle = gamepad1.triangle; circle = gamepad1.circle;
    }

    /** What the camera sees behind the robot right now (INIT only; nothing is locked yet). */
    private String hiveView(IsHiveReadyForShot.Mount mount) {
        int side = settings.side();
        int[] allowed = IsHiveReadyForShot.allowedCells(secondary(), side);
        String prefix = side == 0 ? "" : "Side forced " + (side == 1 ? "RED" : "BLUE") + ": ";
        if (camera == null) return prefix + "NO CAMERA (" + cameraProblem + ") - will not shoot";
        String profile = cameraSettings.overrides().toString() + "/" + side;
        if (initHive == null || !profile.equals(initProfile)) {
            initProfile = profile; initHive = new IsHiveReadyForShot(mount, allowed);
        }
        double now = clock.seconds();
        initHive.update(camera.read(), now);
        return prefix + IsHiveReadyForShot.clusterName(initHive.cluster()) + ": " + initHive.state(now)
                + " / " + initHive.reason;
    }

    @Override public final void start() {
        clock.reset();
        if (fault != null) return;
        try {
            settings.validate();
            localizer.update();
            if (!"READY".equals(PollenOdometryHardware.status(pinpoint))) throw new IllegalStateException("Pinpoint must be READY at START");
            localizer.setPose(Pose.zero());
            IsHiveReadyForShot.Mount mount = IsHiveReadyForShot.Mount.from(cameraSettings);
            hive = new IsHiveReadyForShot(mount, IsHiveReadyForShot.allowedCells(secondary(), settings.side()));
            auto = new PollenHiveAuto(settings, hive, run());
            try { reportFile = PollenHiveStore.newReport(run().toString().toLowerCase(Locale.US)); }
            catch (Exception ex) { reportError = ex.toString(); }
            record("START run=" + run() + " secondary=" + secondary() + " settings=" + settings + " camera=" + cameraSettings.overrides());
            RobotLog.ii("PollenHive", "START build=%s secondary=%s camera=%s changed=[%s] settings=%s | camera %s",
                    PollenConfig.BUILD, secondary(), camera == null ? cameraProblem : "open",
                    join(settings.values.changedSummary(), cameraSettings.changedSummary()), settings,
                    cameraSettings.overrides());
            auto.start(clock.seconds());
        } catch (Exception ex) { fault = ex.toString(); zero(); }
    }

    @Override public final void loop() {
        double now = clock.seconds();
        try {
            if (fault != null || auto == null) { zero(); telemetry.addData("BLOCKED", fault); telemetry.update(); return; }
            localizer.update(); Pose p = localizer.pose();
            double[] pose = PollenFacing.routePose(p.x(), p.y(), p.heading());
            boolean ready = "READY".equals(PollenOdometryHardware.status(pinpoint));
            IsHiveReadyForShot.Frame frame = camera == null ? null : camera.read();
            // DRY predicts feed gating with a virtual flywheel; real motors stay zero.
            double velocity = settings.v[0] == 0 ? (last == null ? 0 : last.launcher) : launcher.getVelocity();
            last = auto.tick(now, frame, pose[0], pose[1], pose[2], velocity, ready);
            if (settings.v[0] == 0) zero(); else {
                double[] wheels = PollenFacing.wheelPowers(last.forward, last.strafe, last.turn);
                for (int i = 0; i < 4; i++) drive[i].setPower(run()==PollenHiveAuto.Run.SHOT_ONLY?0:wheels[i]);
                launcher.setVelocity(settings.v[0] == 2 && run()!=PollenHiveAuto.Run.GARDEN_RETURN ? last.launcher : 0);
                intake.setPower(settings.v[0] == 2 ? last.intake : 0);
                agitator.setPower(settings.v[0] == 2 && run()!=PollenHiveAuto.Run.GARDEN_RETURN ? last.agitator : 0);
            }
            String event = last.phase + " / " + hive.state(now) + " / feed=" + last.feed;
            if (!event.equals(logged)) {
                RobotLog.ii("PollenHive", "t=%.3f %s reason=%s pose=%.2f,%.2f,%.2f", now, event, last.reason, pose[0], pose[1], pose[2]);
                record(String.format(Locale.US, "%s reason=%s poseIn=%.2f,%.2f headingDeg=%.2f", event,last.reason,pose[0],pose[1],Math.toDegrees(pose[2])));
                logged = event;
            }
            // While shooting or waiting to shoot, log what the camera saw 5 times a second.
            boolean shooting = last.phase.startsWith("FIRST") || last.phase.startsWith("SECOND") || last.phase.startsWith("SETTLE");
            if (shooting && now - hiveLogAt >= .2) {
                hiveLogAt = now;
                RobotLog.ii("PollenHive", "t=%.3f HIVE cell=%d tags=%d height=%.1fin sideways=%.1fin state=%s reason=%s pose=%.2f,%.2f,%.1fdeg",
                        now, hive.cluster(), hive.lastTagCount, hive.lastHeightIn, hive.lastSidewaysIn, hive.state(now),
                        hive.reason, pose[0], pose[1], Math.toDegrees(pose[2]));
            }
            if (now - displayAt >= .25) {
                displayAt = now;
                telemetry.addData("Run", run());
                if(!reportError.isEmpty()) telemetry.addData("Report", reportError);
                telemetry.addData("Phase", last.phase);
                telemetry.addData("Our cell", IsHiveReadyForShot.clusterName(hive.cluster()));
                telemetry.addData("Hive / ready", hive.state(now) + " / " + hive.isHiveReadyForShot(now));
                telemetry.addData("Hive height", "%.1f in (%d tags)", hive.lastHeightIn, hive.lastTagCount);
                telemetry.addData("Feed requested / LIVE", last.feed + " / " + (settings.v[0] == 2));
                telemetry.addData("Reason", last.reason);
                telemetry.addData("Camera", camera == null ? "none: " + cameraProblem : camera.status);
                telemetry.addData("Pose fwd / left / heading", "%.1f cm / %.1f cm / %.1f deg", pose[0] * 2.54, pose[1] * 2.54, Math.toDegrees(pose[2]));
                telemetry.addData("Time", now); telemetry.addData("Launcher measured", launcher.getVelocity()); telemetry.update();
            }
        } catch (Exception ex) { fault = ex.toString(); if (auto != null) auto.stop(fault); zero(); }
    }
    private void record(String text) {
        try { if(reportFile!=null) PollenHiveStore.appendReport(reportFile,clock.seconds(),text); }
        catch(Exception ex) { reportError=ex.toString(); }
    }
    private static String join(String a, String b) { return a.isEmpty() ? b : b.isEmpty() ? a : a + ", " + b; }
    private static String nonEmpty(String s, String otherwise) { return s.isEmpty() ? otherwise : s; }
    /** Best effort stops every mapped device even if one disconnected device throws. */
    private void zero() {
        for (DcMotor m : drive) if (m != null) try { m.setPower(0); } catch (RuntimeException ignored) { }
        for (DcMotor m : new DcMotor[]{launcher, intake, agitator}) if (m != null) try { m.setPower(0); } catch (RuntimeException ignored) { }
    }
    @Override public final void stop() { if (auto != null) auto.stop("Driver STOP"); zero(); record("STOP " + (last==null?"before run":last.phase+" "+last.reason)); if (camera != null) camera.stop(); }
}
