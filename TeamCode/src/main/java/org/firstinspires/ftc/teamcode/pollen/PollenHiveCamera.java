package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.json.JSONArray;
import org.json.JSONObject;

/** FTC 12 SDK adapter for the Limelight named "limelight". Reads every hive tag (IDs 30-45) in a
 * frame. Parses the JSON strictly, because the SDK's pose getters turn missing data into zeros. */
public final class PollenHiveCamera {
    private final Limelight3A camera;
    private int pipeline;
    public String status = "Starting";
    public PollenHiveCamera(HardwareMap map, int pipeline) {
        camera = map.get(Limelight3A.class, "limelight");
        camera.setPollRateHz(50);
        setPipeline(pipeline);
        camera.start();
    }
    public int pipeline() { return pipeline; }
    public void setPipeline(int pipeline) {
        this.pipeline = pipeline;
        if (!switchPipeline(camera, pipeline)) throw new IllegalStateException("Limelight rejected pipeline switch");
    }
    /** A pipeline switch is one web request to the camera, and it fails if the camera is busy for
     * a moment. Ask up to three times: one lost request should not leave an auto without its
     * camera, or end a benchmark run. */
    static boolean switchPipeline(Limelight3A camera, int pipeline) {
        for (int attempt = 1; attempt <= SWITCH_ATTEMPTS; attempt++) if (camera.pipelineSwitch(pipeline)) return true;
        return false;
    }
    static final int SWITCH_ATTEMPTS = 3;
    /** Latest fresh frame (possibly with no tags), or null if the camera is stale/disconnected. */
    public IsHiveReadyForShot.Frame read() {
        try {
            LLResult result = camera.getLatestResult();
            if (!camera.isConnected() || result == null || result.getStaleness() < 0 || result.getStaleness() > 150)
                return missing("Disconnected/stale result");
            IsHiveReadyForShot.Frame f = decode(result.toString(), pipeline, result.getStaleness());
            status = "OK, pipeline " + pipeline + ", " + f.tags.length + " hive tag(s)";
            return f;
        } catch (Exception ex) { return missing("Missing/invalid JSON: " + ex.getMessage()); }
    }
    /** Package-private so desktop/JUnit tests can exercise real JSON without opening hardware. */
    static IsHiveReadyForShot.Frame decode(String json, int pipeline, long sdkAgeMs) throws Exception {
        if (sdkAgeMs < 0 || sdkAgeMs > 150) throw new IllegalArgumentException("Stale SDK result");
        JSONObject j = new JSONObject(json);
        if (j.getInt("pID") != pipeline) throw new IllegalArgumentException("Wrong pipeline");
        double capture = j.getDouble("cl"), tracking = j.getDouble("tl");
        if (capture < 0 || tracking < 0) throw new IllegalArgumentException("Negative latency");
        double latency = capture + tracking;
        if (!Double.isFinite(latency) || latency + sdkAgeMs > 200) throw new IllegalArgumentException("Image too old");
        double stamp = j.getDouble("ts");
        List<IsHiveReadyForShot.Tag> tags = new ArrayList<>();
        if (j.getInt("v") == 1) {
            JSONArray targets = j.getJSONArray("Fiducial");
            Set<Integer> seen = new HashSet<>();
            for (int i = 0; i < targets.length(); i++) {
                JSONObject t = targets.getJSONObject(i);
                int id = t.getInt("fID");
                if (IsHiveReadyForShot.clusterOf(id) < 0) continue;
                if (!seen.add(id)) throw new IllegalArgumentException("Duplicate tag ID " + id);
                JSONArray pose = t.getJSONArray("t6t_cs"); // [x, y, z (metres), pitch, yaw, roll (deg)]
                if (pose.length() != 6) throw new IllegalArgumentException("Enable full 3-D AprilTag results");
                for (int k = 0; k < 6; k++) if (!Double.isFinite(pose.getDouble(k))) throw new IllegalArgumentException("Nonfinite pose");
                IsHiveReadyForShot.Tag tag = new IsHiveReadyForShot.Tag(id, pose.getDouble(0), pose.getDouble(1), pose.getDouble(2));
                if (!tag.usable()) throw new IllegalArgumentException("Invalid camera-space pose for tag " + id);
                tags.add(tag);
            }
        }
        IsHiveReadyForShot.Frame f = new IsHiveReadyForShot.Frame(stamp, tags.toArray(new IsHiveReadyForShot.Tag[0]));
        if (!f.usable()) throw new IllegalArgumentException("Invalid timestamp");
        return f;
    }
    private IsHiveReadyForShot.Frame missing(String why) { status = why; return null; }
    public void stop() { camera.stop(); }
}
