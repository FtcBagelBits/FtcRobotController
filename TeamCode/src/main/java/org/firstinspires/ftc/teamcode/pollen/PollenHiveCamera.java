package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.json.JSONArray;
import org.json.JSONObject;

/** FTC 12 SDK adapter. Parse required fields strictly: SDK pose getters default missing data to zero. */
public final class PollenHiveCamera {
    private final Limelight3A camera;
    private int pipeline, tag;
    public String status="Starting";
    public PollenHiveCamera(HardwareMap map, int pipeline, int tag) {
        camera=map.get(Limelight3A.class,"limelight");
        camera.setPollRateHz(50);
        select(pipeline,tag);
        camera.start();
    }
    public void select(int pipeline, int tag) {
        this.pipeline=pipeline; this.tag=tag;
        if(!camera.pipelineSwitch(pipeline)) throw new IllegalStateException("Limelight rejected pipeline switch");
    }
    public IsHiveReadyForShot.Frame read() {
        try {
            LLResult result=camera.getLatestResult();
            if(!camera.isConnected() || result==null || result.getStaleness()<0 || result.getStaleness()>150)
                return missing("Disconnected/stale result");
            IsHiveReadyForShot.Frame f=decode(result.toString(),pipeline,tag,result.getStaleness());
            status="Tag "+tag+" / pipeline "+pipeline;
            return f;
        } catch(Exception ex) { return missing("Missing/invalid JSON: "+ex.getMessage()); }
    }
    /** Package-private so desktop/JUnit tests can exercise real JSON without opening hardware. */
    static IsHiveReadyForShot.Frame decode(String json,int pipeline,int tag,long sdkAgeMs) throws Exception {
        if(sdkAgeMs<0||sdkAgeMs>150) throw new IllegalArgumentException("Stale SDK result");
        JSONObject j=new JSONObject(json);
        if(j.getInt("pID")!=pipeline || j.getInt("v")!=1) throw new IllegalArgumentException("Wrong pipeline/no target");
        double capture=j.getDouble("cl"), tracking=j.getDouble("tl");
        if(capture<0||tracking<0) throw new IllegalArgumentException("Negative latency");
        double latency=capture+tracking;
        if(!Double.isFinite(latency) || latency<0 || latency+sdkAgeMs>200)
            throw new IllegalArgumentException("Image too old");
        double stamp=j.getDouble("ts");
        JSONArray targets=j.getJSONArray("Fiducial");
        JSONObject chosen=null;
        for(int i=0;i<targets.length();i++) {
            JSONObject f=targets.getJSONObject(i);
            if(f.getInt("fID")==tag) {
                if(chosen!=null) throw new IllegalArgumentException("Duplicate tag ID");
                chosen=f;
            }
        }
        if(chosen==null) throw new IllegalArgumentException("Selected tag not visible: "+tag);
        JSONArray pose=chosen.getJSONArray("t6t_cs");
        if(pose.length()!=6) throw new IllegalArgumentException("Enable full 3-D AprilTag results");
        for(int i=0;i<6;i++) if(!Double.isFinite(pose.getDouble(i))) throw new IllegalArgumentException("Nonfinite pose");
        IsHiveReadyForShot.Frame f=new IsHiveReadyForShot.Frame(stamp,pose.getDouble(0),
                pose.getDouble(1),pose.getDouble(2),true);
        if(!f.usable()) throw new IllegalArgumentException("Invalid camera-space pose");
        return f;
    }
    private IsHiveReadyForShot.Frame missing(String why) { status=why; return null; }
    public void stop() { camera.stop(); }
}
