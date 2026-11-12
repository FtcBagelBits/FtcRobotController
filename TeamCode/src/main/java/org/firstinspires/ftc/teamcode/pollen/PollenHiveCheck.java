package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import java.io.File;
import java.util.Locale;

/** Camera setup, labeled checks and continuous readiness display. Never maps a motor or odometry. */
@Autonomous(name = "TEST Hive camera (NO MOTORS)", group = "Pollen Hive")
public final class PollenHiveCheck extends OpMode {
    private final ElapsedTime clock = new ElapsedTime();
    private final PollenHiveSettings.Values settings = new PollenHiveSettings.Values(PollenHiveSettings.CAMERA, false);
    private PollenHiveCalibrationSession session;
    private PollenHiveCamera camera;
    private IsHiveReadyForShot detector;
    private PollenHiveCalibration capture;
    private IsHiveReadyForShot.State expected = IsHiveReadyForShot.State.UP;
    private double captureAt, displayAt = -1;
    private int row;
    private boolean square, circle, triangle, cross, up, down, left, right;
    private String message = "", error = "";
    private String[] report = new String[0];
    private File reportFile;

    @Override public void init() { message = PollenHiveStore.load(PollenHiveStore.CAMERA, settings); }
    @Override public void init_loop() {
        if (gamepad1.dpad_up && !up) row = (row + settings.v.length - 1) % settings.v.length;
        if (gamepad1.dpad_down && !down) row = (row + 1) % settings.v.length;
        int delta = (gamepad1.dpad_right && !right ? 1 : 0) - (gamepad1.dpad_left && !left ? 1 : 0);
        if (delta != 0) settings.edit(row, delta);
        if (gamepad1.triangle && !triangle) settings.reset(row);
        if (gamepad1.square && !square) {
            try { PollenHiveStore.save(PollenHiveStore.CAMERA, settings); message = "Saved camera rows; validate after START"; }
            catch (Exception ex) { message = "NOT SAVED: " + ex.getMessage(); }
        }
        buttons();
        telemetry.addData("BUILD", PollenConfig.BUILD);
        telemetry.addLine("NO MOTORS. D-pad selects/edits camera rows. Triangle resets ONE row. Square saves.");
        for (int i = 0; i < settings.v.length; i++) telemetry.addLine((row == i ? "> " : "  ") + settings.line(i));
        telemetry.addData("Status", message);
        telemetry.addLine("Measure floor to lens center and UP tag-row center. START for calibration/checks.");
        telemetry.update();
    }
    @Override public void start() {
        clock.reset();
        session = new PollenHiveCalibrationSession(IsHiveReadyForShot.Mount.from(settings));
        forgetDetector();
        try { camera = new PollenHiveCamera(hardwareMap, (int) settings.get("pipeline")); }
        catch (Exception ex) { error = ex.toString(); }
        try { reportFile = PollenHiveStore.newReport("camera"); }
        catch (Exception ex) { error += " Report unavailable: " + ex.getMessage(); }
        record("START camera=" + settings.overrides());
        message = "Physical cell UP: Square fits tilt. Already calibrated? Circle checks UP.";
    }
    @Override public void loop() {
        double now = clock.seconds();
        IsHiveReadyForShot.Frame f = camera == null ? null : camera.read();
        if (capture == null) {
            if (gamepad1.dpad_left && !left) expected = IsHiveReadyForShot.State.UP;
            if (gamepad1.dpad_right && !right) expected = IsHiveReadyForShot.State.DOWN;
            if (gamepad1.triangle && !triangle) {
                session.resetChecks(); session.cluster = -1; forgetDetector();
                message = "Cell forgotten; confirm the new cell name before Circle";
            } else if (gamepad1.square && !square) {
                session.beginTilt(); expected = IsHiveReadyForShot.State.UP; forgetDetector();
                capture = new PollenHiveCalibration(true, session.live, expected, -1);
                captureAt = now; report = new String[0]; message = "Fitting tilt: PRIMARY start, cell physically UP, hold still";
            } else if (gamepad1.circle && !circle) {
                if (!session.live.problem().isEmpty()) message = session.live.problem();
                else if (detector.cluster() < 0) message = "First identify our cell while UP. Then check both states without moving.";
                else {
                    // A new check revokes the old result before collecting any evidence.
                    if (expected == IsHiveReadyForShot.State.UP) session.upChecked = false; else session.downChecked = false;
                    capture = new PollenHiveCalibration(false, session.live, expected, detector.cluster());
                    captureAt = now; report = new String[0]; message = "Checking PHYSICAL " + expected + ": hold still";
                }
            } else if (gamepad1.cross && !cross) save();
        }
        buttons();
        detector.update(f, now);
        if (capture != null) {
            capture.add(f, now);
            if (now - captureAt >= PollenHiveCalibration.SECONDS) finish();
        }
        if (now - displayAt >= .25) {
            displayAt = now;
            telemetry.addData("BUILD", PollenConfig.BUILD);
            telemetry.addData("Profile", (session.candidate ? "CANDIDATE " : "CURRENT ") + session.version + ": " + describe(session.live));
            telemetry.addData("Our cell", IsHiveReadyForShot.clusterName(detector.cluster()));
            telemetry.addData("State / IsHiveReadyForShot", detector.state(now) + " / " + detector.isHiveReadyForShot(now));
            telemetry.addData("Detector", detector.reason);
            telemetry.addData("PHYSICAL check label", expected + " (left=UP, right=DOWN)");
            telemetry.addData("Validated", "UP=" + session.upChecked + " DOWN/blocked=" + session.downChecked);
            telemetry.addData("Capture", capture == null ? "idle" : capture.frames + " accepted / " + capture.rejected + " rejected");
            telemetry.addData("Tags", tags(f));
            telemetry.addData("Camera", error.isEmpty() ? camera == null ? "Not open" : camera.status : error);
            telemetry.addData("Status", message);
            for (String line : report) telemetry.addLine(line);
            telemetry.addLine("Square=fit UP tilt; Circle=check label; Cross=save validated; Triangle=forget cell.");
            telemetry.addLine("Continuous CAMERA gate only: no launcher-speed or robot-position test. Runs until STOP.");
            telemetry.update();
            RobotLog.ii("PollenHive", "CHECK state=%s ready=%s cell=%d height=%.2f expected=%s reason=%s",
                    detector.state(now), detector.isHiveReadyForShot(now), detector.cluster(), detector.lastHeightIn, expected, detector.reason);
        }
    }
    private void finish() {
        report = capture.report();
        boolean ok = session.accept(capture);
        for (String line : report) record("version=" + session.version + " " + line);
        if (capture.tilt && ok) {
            detector = new IsHiveReadyForShot(session.live, session.cluster);
            message = "Candidate in use. Circle checks physical UP; tip DOWN, right, Circle; Cross saves.";
        } else if (ok) message = capture.blockedOnly() ? "DOWN hidden: blocking verified, height NOT verified" : "Check passed";
        else message = "Capture rejected; fix the cause and repeat";
        capture = null;
    }
    private void save() {
        if (!session.canSave()) { message = "Need successful physical UP and DOWN/blocked checks of this profile and cell"; return; }
        settings.v[settings.index("tilt")] = session.live.tiltUpDeg;
        settings.v[settings.index("sideways")] = session.live.sidewaysCorrectionIn;
        try {
            PollenHiveStore.save(PollenHiveStore.CAMERA, settings); session.saved();
            message = "SAVED. STOP / INIT reloads for a persistence check. All hive modes use this profile.";
            record("SAVED version=" + session.version + " camera=" + settings.overrides());
        } catch (Exception ex) { message = "NOT SAVED: " + ex.getMessage(); }
    }
    private void forgetDetector() { detector = new IsHiveReadyForShot(session.live, IsHiveReadyForShot.ALL_CELLS); }
    private void buttons() {
        square = gamepad1.square; circle = gamepad1.circle; triangle = gamepad1.triangle; cross = gamepad1.cross;
        up = gamepad1.dpad_up; down = gamepad1.dpad_down; left = gamepad1.dpad_left; right = gamepad1.dpad_right;
    }
    private void record(String text) {
        RobotLog.ii("PollenHive", "%s", text);
        try { if (reportFile != null) PollenHiveStore.appendReport(reportFile, clock.seconds(), text); }
        catch (Exception ex) { error = "Report write failed: " + ex.getMessage(); }
    }
    private String tags(IsHiveReadyForShot.Frame f) {
        if (f == null) return "none (camera missing/invalid)";
        StringBuilder b = new StringBuilder();
        for (IsHiveReadyForShot.Tag t : f.tags) b.append(String.format(Locale.US, "%d: %.1fin high, %.1fin side; ",
                t.id, session.live.heightIn(t), session.live.sidewaysIn(t)));
        return b.length() == 0 ? "no hive tags" : b.toString();
    }
    private static String describe(IsHiveReadyForShot.Mount m) {
        return String.format(Locale.US, "lens %.2fin tilt %.1fdeg side %.2fin UP %.1f DOWN %.1f", m.lensHeightIn,
                m.tiltUpDeg, m.sidewaysCorrectionIn, m.upIn, m.downIn);
    }
    @Override public void stop() {
        record("STOP " + (session == null ? "before START" : "version=" + session.version + " UP=" + session.upChecked + " DOWN/blocked=" + session.downChecked));
        if (camera != null) camera.stop();
    }
}
