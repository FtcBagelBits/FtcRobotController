package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.*;
import org.json.JSONArray;
import org.json.JSONObject;

/** No motors, odometry, auto permission changes, or camera-setting writes. Only switches saved
 * pipelines. Buffered JSONL includes every sampled result, including stale/invalid/empty polls. */
@Autonomous(name = "TEST Hive pipeline benchmark", group = "Pollen Hive")
public final class PollenHiveBenchmark extends OpMode {
    private final ElapsedTime clock = new ElapsedTime();
    private final PollenHiveSettings.Values settings = new PollenHiveSettings.Values(PollenHiveSettings.CAMERA, false);
    private final String[] names = {"First pipeline", "Last pipeline", "Passes (UP + DOWN)", "Seconds per pipeline", "Our cell first ID", "Scenario"};
    private final int[] values = {0, 0, 2, 20, 34, 0};
    private final int[] min = {0, 0, 1, 5, 30, 0}, max = {9, 9, 5, 60, 42, 5}, step = {1, 1, 1, 5, 4, 1};
    private static final String[] SCENARIOS = {"normal", "secondary-start", "shift-left", "shift-right", "occluded", "other"};
    private Limelight3A camera;
    private PollenHiveBenchmarkPlan plan;
    private IsHiveReadyForShot detector;
    private IsHiveReadyForShot.Mount mount;
    private BufferedWriter writer;
    private File file;
    private int row, polls, fresh, usable, rawHits, sequence;
    private long bytes;
    private double nextSample, nextDisplay, flushAt, lastStamp = Double.NaN;
    private boolean left, right, up, down, cancelled;
    private String message = "", decodeError = "", restore = "not needed";
    private int originalPipeline;
    private boolean switched, detectorSeeded;
    @Override public void init() {
        message = PollenHiveStore.load(PollenHiveStore.CAMERA, settings);
        values[0] = values[1] = (int) settings.get("pipeline");
    }
    @Override public void init_loop() {
        if (gamepad1.dpad_up && !up) row = (row + names.length - 1) % names.length;
        if (gamepad1.dpad_down && !down) row = (row + 1) % names.length;
        int delta = (gamepad1.dpad_right && !right ? 1 : 0) - (gamepad1.dpad_left && !left ? 1 : 0);
        values[row] = Math.max(min[row], Math.min(max[row], values[row] + delta * step[row]));
        buttons();
        telemetry.addData("BUILD", PollenConfig.BUILD);
        telemetry.addLine("CAMERA ONLY. D-pad row/value. Configure saved pipeline slots first.");
        for (int i = 0; i < names.length; i++) telemetry.addData((i == row ? "> " : "") + names[i],
                i == 5 ? SCENARIOS[values[i]] : Integer.toString(values[i]));
        telemetry.addData("Cell", IsHiveReadyForShot.clusterName(values[4]));
        telemetry.addData("Camera settings", message);
        telemetry.addData("Readiness calibration", IsHiveReadyForShot.Mount.from(settings).problem());
        telemetry.addLine("Uncalibrated? Raw detection still records. Readiness will be UNKNOWN.");
        telemetry.addLine("START waits for human UP label; nothing moves or fires.");
        telemetry.update();
    }
    @Override public void start() {
        clock.reset();
        mount = IsHiveReadyForShot.Mount.from(settings);
        originalPipeline = (int) settings.get("pipeline");
        try {
            plan = new PollenHiveBenchmarkPlan(values[0], values[1], values[2], values[3]);
            File dir = new File(PollenHiveStore.folder(), "benchmark");
            if (!dir.isDirectory() && !dir.mkdirs()) throw new IOException("Cannot create " + dir);
            file = File.createTempFile("hive-" + System.currentTimeMillis() + "-", ".jsonl", dir);
            writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(file), "UTF-8"), 65536);
            JSONObject config = new JSONObject();
            for (int i = 0; i < settings.rows.length; i++) config.put(settings.rows[i].key, finite(settings.v[i]));
            write(new JSONObject().put("event", "session").put("schema", 1).put("build", PollenConfig.BUILD)
                    .put("wall_ms", System.currentTimeMillis()).put("camera", config).put("cell", values[4])
                    .put("scenario", SCENARIOS[values[5]]).put("first", values[0]).put("last", values[1])
                    .put("passes", values[2]).put("seconds", values[3]).put("poll_hz", 50)
                    .put("confirm_frames", IsHiveReadyForShot.CONFIRM_FRAMES)
                    .put("confirm_seconds", IsHiveReadyForShot.CONFIRM_SECONDS).put("max_age", IsHiveReadyForShot.MAX_AGE));
            camera = hardwareMap.get(Limelight3A.class, "limelight");
            camera.setPollRateHz(50); camera.start();
            message = "Recording ready. Follow the physical-label prompt; hold still through each block.";
        } catch (Exception ex) { fail(ex); }
    }
    @Override public void loop() {
        if (plan == null) { display(); return; }
        double now = clock.seconds();
        try {
            if (gamepad1.circle && plan.phase != PollenHiveBenchmarkPlan.Phase.DONE) {
                cancelled = true; plan.fail("Cancelled: this run is incomplete. STOP; restart for new labels.");
            }
            if (plan.phase == PollenHiveBenchmarkPlan.Phase.WAIT) {
                if (gamepad1.dpad_left && !left) plan.confirm("UP", now);
                if (gamepad1.dpad_right && !right) plan.confirm("DOWN", now);
            }
            buttons();
            if (plan.phase == PollenHiveBenchmarkPlan.Phase.SWITCH) {
                switched = true;
                boolean ok = camera.pipelineSwitch(plan.pipeline());
                plan.switched(ok, clock.seconds());
                if (!ok) throw new IOException("Camera rejected pipeline " + plan.pipeline());
                detector = new IsHiveReadyForShot(mount, values[4]);
                detectorSeeded = false; lastStamp = Double.NaN; polls = fresh = usable = rawHits = 0;
                event("trial", clock.seconds());
            }
            now = clock.seconds();
            PollenHiveBenchmarkPlan.Phase before = plan.phase;
            if (before == PollenHiveBenchmarkPlan.Phase.RECORD && now - plan.since >= plan.seconds)
                event("record_end", now);
            plan.tick(now);
            if (before != plan.phase) {
                if (plan.phase == PollenHiveBenchmarkPlan.Phase.RECORD) {
                    // Exclude switch/settle observations; a repeated cached frame cannot start confirmation.
                    detector = new IsHiveReadyForShot(mount, values[4]);
                    detectorSeeded = false; polls = fresh = usable = rawHits = 0;
                    event("record_start", now);
                } else if (before == PollenHiveBenchmarkPlan.Phase.RECORD) {
                    writer.flush();
                }
            }
            if ((plan.phase == PollenHiveBenchmarkPlan.Phase.SETTLE || plan.phase == PollenHiveBenchmarkPlan.Phase.RECORD)
                    && now >= nextSample) {
                nextSample = now + .02;
                sample(now);
            }
            if (now - flushAt >= 1 && writer != null) { writer.flush(); flushAt = now; }
            if (bytes > 250L * 1024 * 1024) throw new IOException("250 MiB run limit reached; STOP and download");
            if (plan.phase == PollenHiveBenchmarkPlan.Phase.DONE || plan.phase == PollenHiveBenchmarkPlan.Phase.ERROR) restore();
        } catch (Exception ex) { fail(ex); }
        if (now >= nextDisplay) { nextDisplay = now + .25; display(); }
    }
    private void sample(double now) throws Exception {
        LLResult r = camera.getLatestResult();
        long age = r == null ? -1 : r.getStaleness();
        JSONObject raw = null;
        String text = r == null ? "" : r.toString();
        try { if (r != null) raw = new JSONObject(text); } catch (Exception ignored) { }
        double stamp = raw == null ? Double.NaN : raw.optDouble("ts", Double.NaN);
        boolean distinct = Double.isFinite(stamp) && stamp > 0 && stamp != lastStamp;
        boolean correctPipeline = raw != null && raw.optInt("pID", -1) == plan.pipeline();
        IsHiveReadyForShot.Frame frame = null;
        decodeError = "";
        try {
            if (r == null || !camera.isConnected()) throw new IllegalArgumentException("Disconnected/no SDK result");
            frame = PollenHiveCamera.decode(text, plan.pipeline(), age);
        } catch (Exception ex) { decodeError = ex.getMessage(); }
        // Seed a new detector only with a new expected-pipeline frame. Thereafter let production
        // duplicate/stale handling operate normally (duplicates do not count as new evidence).
        if (distinct && correctPipeline) lastStamp = stamp;
        if (distinct && correctPipeline && frame != null) detectorSeeded = true;
        detector.update(detectorSeeded ? frame : null, now);
        JSONArray derived = new JSONArray();
        if (frame != null) for (IsHiveReadyForShot.Tag tag : frame.tags) {
            derived.put(new JSONObject().put("id", tag.id).put("height_in", finite(mount.heightIn(tag)))
                    .put("side_in", finite(mount.sidewaysIn(tag))).put("our_cell", IsHiveReadyForShot.clusterOf(tag.id) == values[4])
                    .put("inside_side_gate", Math.abs(mount.sidewaysIn(tag)) <= mount.oursMaxSidewaysIn));
        }
        boolean rawHit = false;
        JSONArray fiducials = raw == null ? null : raw.optJSONArray("Fiducial");
        if (fiducials != null) for (int i = 0; i < fiducials.length(); i++) {
            JSONObject t = fiducials.optJSONObject(i);
            if (t != null && IsHiveReadyForShot.clusterOf(t.optInt("fID", -1)) == values[4]) rawHit = true;
        }
        boolean accepted = frame != null && IsHiveReadyForShot.nearestCluster(frame, mount, values[4]) == values[4];
        polls++;
        if (distinct && correctPipeline) { fresh++; if (rawHit) rawHits++; if (accepted) usable++; }
        write(new JSONObject().put("event", "sample").put("t", now).put("seq", sequence++)
                .put("trial", plan.trial()).put("phase", plan.phase.toString()).put("expected", plan.expected())
                .put("pipeline", plan.pipeline()).put("connected", camera.isConnected()).put("sdk_age_ms", age)
                .put("distinct", distinct && correctPipeline).put("decode_error", decodeError)
                .put("accepted", accepted).put("raw_cell_hit", rawHit).put("state", detector.state(now).toString())
                .put("reason", detector.reason).put("tags", derived).put("raw", raw == null ? JSONObject.NULL : raw)
                .put("raw_text", raw == null ? text : ""));
    }
    private void event(String kind, double now) throws Exception {
        write(new JSONObject().put("event", kind).put("t", now).put("trial", plan.trial())
                .put("pipeline", plan.pipeline()).put("expected", plan.expected()).put("pass", plan.pass()));
    }
    private static Object finite(double d) { return Double.isFinite(d) ? d : JSONObject.NULL; }
    private void write(JSONObject obj) throws IOException {
        if (writer == null) throw new IOException("Log is not open");
        String s = obj.toString(); writer.write(s); writer.newLine(); bytes += s.length() * 2L;
    }
    private void buttons() { left = gamepad1.dpad_left; right = gamepad1.dpad_right; up = gamepad1.dpad_up; down = gamepad1.dpad_down; }
    private void restore() {
        if (!switched || camera == null) return;
        switched = false;
        restore = camera.pipelineSwitch(originalPipeline) ? "restored saved pipeline " + originalPipeline : "FAILED: select saved pipeline manually";
    }
    private void fail(Exception ex) {
        message = "STOP: " + ex.toString();
        if (plan != null) plan.fail(message);
        // Never continue an unrecorded benchmark after an I/O error.
    }
    private void display() {
        telemetry.addData("BUILD", PollenConfig.BUILD);
        telemetry.addData("Phase", plan == null ? "ERROR" : plan.phase);
        if (plan != null) {
            telemetry.addData("Physical label", plan.expected() + " / " + IsHiveReadyForShot.clusterName(values[4]));
            if (plan.phase == PollenHiveBenchmarkPlan.Phase.WAIT)
                telemetry.addLine("MOVE ONLY NOW; let hive settle. " + (plan.expected().equals("UP") ? "LEFT confirms UP" : "RIGHT confirms DOWN"));
            else if (plan.phase == PollenHiveBenchmarkPlan.Phase.RECORD || plan.phase == PollenHiveBenchmarkPlan.Phase.SETTLE) {
                telemetry.addData("Hold still", "pipeline " + plan.pipeline() + "; pass " + plan.pass() + "/" + plan.repeats);
                telemetry.addData("Seconds remaining", Math.max(0, (plan.phase == PollenHiveBenchmarkPlan.Phase.SETTLE ? 2 : plan.seconds) - (clock.seconds() - plan.since)));
            }
            telemetry.addData("Fresh/raw cell/usable", fresh + " / " + rawHits + " / " + usable);
            telemetry.addData("Last rejection", decodeError);
            if (detector != null) telemetry.addData("Readiness", detector.state(clock.seconds()) + ": " + detector.reason);
            telemetry.addData("Error", plan.error);
        }
        telemetry.addData("Log", file == null ? "NOT OPEN" : file.getName());
        telemetry.addData("Status", message);
        telemetry.addData("Pipeline restore", restore);
        telemetry.addLine("Circle cancels mislabeled run. DONE: STOP, download logs. No motors mapped.");
        telemetry.update();
    }
    @Override public void stop() {
        try { restore(); } catch (Exception ex) { restore = ex.toString(); }
        if (camera != null) camera.stop();
        if (writer != null) {
            try {
                write(new JSONObject().put("event", "stop").put("t", clock.seconds()).put("complete",
                        !cancelled && plan != null && plan.phase == PollenHiveBenchmarkPlan.Phase.DONE)
                        .put("restore", restore).put("message", message).put("plan_error", plan == null ? "No plan" : plan.error));
                writer.close();
            } catch (Exception ex) { telemetry.addData("LOG CLOSE FAILED", ex.toString()); telemetry.update(); }
            writer = null;
        }
    }
}
