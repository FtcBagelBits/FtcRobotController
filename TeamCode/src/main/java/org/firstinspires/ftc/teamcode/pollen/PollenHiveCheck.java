package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.Locale;

/** Camera calibration and checks. Never maps or commands a motor. Nothing to choose in INIT.
 * After START:
 *   Square   = 3 s TILT capture (robot at the PRIMARY start, our cell UP, nothing moving).
 *              The result is a CANDIDATE, used on screen but not saved.
 *   Circle   = 3 s HEIGHT check of the cell behind the robot (UP or DOWN), with the candidate if any.
 *   Cross    = SAVE the candidate (tilt, sideways, and a DOWN height a Circle check suggested) on the
 *              Hub as camera changes. Every hive mode uses them from its next INIT.
 *   Triangle = forget which cell is ours (press after moving the robot to another start spot).
 * Results also go to the robot log (search PollenHive). Edit any camera row in either hive auto's
 * INIT menu; promote proven changes to code defaults with tools/hive-promote-defaults.py. */
@Autonomous(name = "TEST Hive camera (NO MOTORS)", group = "Pollen Hive")
public final class PollenHiveCheck extends OpMode {
    private final ElapsedTime clock = new ElapsedTime();
    private final PollenHiveSettings.Values cameraSettings = new PollenHiveSettings.Values(PollenHiveSettings.CAMERA, false);
    private IsHiveReadyForShot.Mount code, live; // code = saved settings (defaults + Hub changes)
    private String loadMessage;
    private double suggestedDown = Double.NaN;
    private boolean prevCross;
    private PollenHiveCamera camera;
    private IsHiveReadyForShot detector;
    private PollenHiveCalibration capture;
    private double captureAt, lastDisplay = -10;
    private boolean candidate, prevSquare, prevCircle, prevTriangle;
    private String error, message = "Square = tilt capture, Circle = height check, Triangle = forget cell";
    private String[] report = new String[0];

    @Override public void init() {
        loadMessage = PollenHiveStore.load(PollenHiveStore.CAMERA, cameraSettings);
        code = live = IsHiveReadyForShot.Mount.from(cameraSettings);
        detector = new IsHiveReadyForShot(code, IsHiveReadyForShot.ALL_CELLS);
    }

    @Override public void init_loop() {
        telemetry.addData("BUILD", PollenConfig.BUILD);
        telemetry.addLine("NO MOTORS. Nothing to choose. Press START, then follow the hive calibration sheet.");
        telemetry.addData("Camera settings", loadMessage);
        for (int i = 0; i < cameraSettings.v.length; i++) telemetry.addLine("  " + cameraSettings.line(i));
        telemetry.update();
    }

    @Override public void start() {
        clock.reset();
        try { camera = new PollenHiveCamera(hardwareMap, (int) cameraSettings.get("pipeline")); }
        catch (Exception ex) { error = ex.toString(); }
    }

    @Override public void loop() {
        double now = clock.seconds();
        IsHiveReadyForShot.Frame f = camera == null ? null : camera.read();
        if (gamepad1.triangle && !prevTriangle) { detector = new IsHiveReadyForShot(live, IsHiveReadyForShot.ALL_CELLS); message = "Forgot our cell; looking again"; }
        if (gamepad1.cross && !prevCross && capture == null) save();
        prevCross = gamepad1.cross;
        if (gamepad1.square && !prevSquare && capture == null) begin(true, now);
        if (gamepad1.circle && !prevCircle && capture == null) {
            if (!live.problem().isEmpty()) message = "Height check needs a tilt: press Square first. " + live.problem();
            else begin(false, now);
        }
        prevSquare = gamepad1.square; prevCircle = gamepad1.circle; prevTriangle = gamepad1.triangle;
        if (capture != null) {
            capture.add(f);
            if (now - captureAt >= PollenHiveCalibration.SECONDS) finish();
        }
        detector.update(f, now);
        if (now - lastDisplay >= .5) {
            lastDisplay = now;
            telemetry.addData("Using", (candidate ? "CANDIDATE (Cross saves): " : "SAVED/DEFAULT: ") + describe(live));
            telemetry.addData("Our cell", IsHiveReadyForShot.clusterName(detector.cluster()));
            telemetry.addData("State / ready", detector.state(now) + " / " + detector.isHiveReadyForShot(now));
            telemetry.addData("Detector", detector.reason);
            telemetry.addData("Tags", tags(f));
            telemetry.addData("Camera", error != null ? error : camera == null ? "Not open" : camera.status);
            telemetry.addData("Capture", capture == null ? "idle" : (capture.tilt ? "TILT " : "HEIGHT ") + capture.frames + " frames");
            telemetry.addData("Status", message);
            for (String line : report) telemetry.addLine(line);
            telemetry.addLine("Square=TILT  Circle=HEIGHT check  Cross=SAVE candidate  Triangle=forget cell. No motors.");
            telemetry.update();
            RobotLog.ii("PollenHive", "CHECK cell=%d state=%s height=%.1fin sideways=%.1fin reason=%s tags=%s",
                    detector.cluster(), detector.state(now), detector.lastHeightIn, detector.lastSidewaysIn, detector.reason, tags(f));
        }
    }

    private void begin(boolean tilt, double now) {
        // A tilt capture starts from the saved numbers (lens height, UP height, sideways) and finds the tilt.
        capture = new PollenHiveCalibration(tilt, tilt ? code : live);
        captureAt = now; report = new String[0];
        message = tilt ? "TILT: robot at PRIMARY start, our cell UP, hold still 3 s" : "HEIGHT: hold still 3 s";
    }

    private void finish() {
        report = capture.report();
        boolean ok = capture.problem().isEmpty();
        if (capture.tilt && ok) {
            double s = capture.sidewaysCorrectionIn();
            live = new IsHiveReadyForShot.Mount(code.lensHeightIn, capture.tiltDeg(),
                    Double.isNaN(s) ? code.sidewaysCorrectionIn : s, code.upIn, code.downIn, code.bandIn, code.oursMaxSidewaysIn);
            candidate = true;
            detector = new IsHiveReadyForShot(live, IsHiveReadyForShot.ALL_CELLS);
            message = "Candidate in use. Check with Circle (UP here, then DOWN), then Cross to save.";
        } else if (!capture.tilt && ok) {
            double d = capture.suggestedDownIn();
            if (!Double.isNaN(d)) { suggestedDown = d; candidate = true; message = "Height check done; Cross saves the new DOWN height"; }
            else message = "Height check done";
        } else message = "Capture rejected";
        for (String line : report) RobotLog.ii("PollenHive", "CALIBRATION %s %s", capture.tilt ? "TILT" : "HEIGHT", line);
        capture = null;
    }

    /** Cross: write the candidate numbers into the camera settings and save them on the Hub. */
    private void save() {
        if (!candidate) { message = "Nothing to save: press Square (and Circle) first"; return; }
        cameraSettings.v[cameraSettings.index("tilt")] = Math.round(live.tiltUpDeg * 10) / 10.0;
        cameraSettings.v[cameraSettings.index("sideways")] = Math.round(live.sidewaysCorrectionIn * 100) / 100.0;
        if (!Double.isNaN(suggestedDown)) cameraSettings.v[cameraSettings.index("downHeight")] = suggestedDown;
        try {
            PollenHiveStore.save(PollenHiveStore.CAMERA, cameraSettings);
            code = live = IsHiveReadyForShot.Mount.from(cameraSettings);
            candidate = false; suggestedDown = Double.NaN;
            detector = new IsHiveReadyForShot(live, IsHiveReadyForShot.ALL_CELLS);
            message = "SAVED on the Hub: " + cameraSettings.changedSummary() + ". Hive autos use it from their next INIT.";
            RobotLog.ii("PollenHive", "CALIBRATION SAVED camera: %s", cameraSettings.overrides());
        } catch (Exception ex) { message = "NOT SAVED: " + ex.getMessage(); }
    }

    private String tags(IsHiveReadyForShot.Frame f) {
        if (f == null) return "none";
        StringBuilder b = new StringBuilder();
        for (IsHiveReadyForShot.Tag t : f.tags) {
            b.append(b.length() == 0 ? "" : " | ").append(t.id).append(": ");
            if (live.problem().isEmpty()) b.append(String.format(Locale.US, "%.1f in high, ", live.heightIn(t)));
            b.append(String.format(Locale.US, "%.1f in side", live.sidewaysIn(t)));
        }
        return b.length() == 0 ? "none in view" : b.toString();
    }

    private static String describe(IsHiveReadyForShot.Mount m) {
        return String.format(Locale.US, "lens %.1f in, tilt %s, sideways %.1f in, UP %.1f, DOWN %.1f",
                m.lensHeightIn, Double.isNaN(m.tiltUpDeg) ? "NOT SET" : String.format(Locale.US, "%.1f deg", m.tiltUpDeg),
                m.sidewaysCorrectionIn, m.upIn, m.downIn);
    }

    @Override public void stop() { if (camera != null) camera.stop(); }
}
