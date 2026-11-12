package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/** Never maps or commands a motor. START enters calibration/observation, still with no motors. */
@Autonomous(name="TEST Hive camera (NO MOTORS)",group="Pollen Hive")
public final class PollenHiveCheck extends OpMode {
    private final ElapsedTime clock=new ElapsedTime();
    private PollenHiveCamera camera;
    private IsHiveReadyForShot detector;
    private IsHiveReadyForShot.Box up,down;
    private IsHiveReadyForShot.Capture capture;
    private String label="", message="No profile loaded", error;
    private int slot=1,pipeline=0,tag=34,row;
    private boolean prevUp,prevDown,prevLeft,prevRight,prevSquare,prevCircle,prevCross,prevTriangle;
    private double captureAt,lastDisplay=-10;
    @Override public void init() { telemetry.addLine("NO MOTORS. Select slot/pipeline/tag during INIT."); telemetry.update(); }
    @Override public void init_loop() {
        if(gamepad1.dpad_up&&!prevUp) row=(row+2)%3;
        if(gamepad1.dpad_down&&!prevDown) row=(row+1)%3;
        int delta=(gamepad1.dpad_right&&!prevRight?1:0)-(gamepad1.dpad_left&&!prevLeft?1:0);
        if(row==0) slot=Math.max(1,Math.min(8,slot+delta));
        if(row==1) pipeline=Math.max(0,Math.min(9,pipeline+delta));
        if(row==2) tag=Math.max(0,Math.min(586,tag+delta));
        telemetry.addData("BUILD",PollenConfig.BUILD);
        telemetry.addData("Select row (dpad up/down)",row);
        telemetry.addData("0 slot / 1 pipeline / 2 tag", "%d / %d / %d",slot,pipeline,tag);
        telemetry.addLine("After START: Square capture UP; Circle DOWN; Cross save; Triangle clear candidate.");
        telemetry.addLine("Selected slot's saved profile loads if its pipeline/tag match.");
        telemetry.update(); buttons();
    }
    @Override public void start() {
        clock.reset();
        try {
            camera=new PollenHiveCamera(hardwareMap,pipeline,tag);
            try {
                IsHiveReadyForShot.Profile p=PollenHiveFiles.loadProfile(slot);
                if(p.pipeline!=pipeline || p.tag!=tag) throw new IllegalArgumentException("Saved profile uses different tag/pipeline");
                up=p.up; down=p.down; detector=new IsHiveReadyForShot(p); message="Loaded slot "+slot;
            } catch(Exception ex) { message="No matching profile: "+ex.getMessage(); }
        } catch(Exception ex) { error=ex.toString(); }
    }
    @Override public void loop() {
        double now=clock.seconds();
        IsHiveReadyForShot.Frame f=camera==null?null:camera.read();
        if(gamepad1.triangle&&!prevTriangle) { up=down=null; detector=null; capture=null; message="Candidate cleared; disk unchanged until SAVE"; }
        if(gamepad1.square&&!prevSquare) begin("UP",now);
        if(gamepad1.circle&&!prevCircle) begin("DOWN",now);
        if(capture!=null) {
            capture.add(f);
            if(now-captureAt>=5) {
                try {
                    PollenHiveFiles.saveCapture(slot,label,pipeline,tag,capture);
                    IsHiveReadyForShot.Box box=capture.finish();
                    if(label.equals("UP")) up=box; else down=box;
                    detector=new IsHiveReadyForShot(new IsHiveReadyForShot.Profile(pipeline,tag,up,down));
                    message="Captured "+label+"; Cross saves. Validate separately with repeated tips.";
                } catch(Exception ex) { detector=null; message="Capture rejected: "+ex.getMessage(); }
                capture=null;
            }
        }
        if(gamepad1.cross&&!prevCross && capture==null) {
            try {
                IsHiveReadyForShot.Profile p=new IsHiveReadyForShot.Profile(pipeline,tag,up,down);
                PollenHiveFiles.saveProfile(slot,p); detector=new IsHiveReadyForShot(p);
                message="SAVED slot "+slot+". "+(down==null?"UP-only: DOWN will report UNKNOWN.":"UP/DOWN.");
            } catch(Exception ex) { message="SAVE FAILED: "+ex.getMessage(); }
        }
        if(detector!=null) detector.update(f,now);
        if(now-lastDisplay>=1) {
            lastDisplay=now;
            String state=detector==null?"UNKNOWN":detector.state(now).toString();
            telemetry.addData("State / ready",state+" / "+(detector!=null&&detector.isHiveReadyForShot(now)));
            telemetry.addData("Detector",detector==null?"Uncalibrated/capturing":detector.reason);
            telemetry.addData("Slot / tag / pipeline","%d / %d / %d",slot,tag,pipeline);
            telemetry.addData("Camera",error!=null?error:camera==null?"Not open":camera.status);
            if(f!=null) telemetry.addData("Camera x/y/z METRES","%.3f / %.3f / %.3f",f.x,f.y,f.z);
            telemetry.addData("Capture",capture==null?"Idle":label+" frames="+capture.count+" / 5 seconds");
            telemetry.addData("Status",message);
            telemetry.addLine("Square=UP, Circle=DOWN, Cross=SAVE, Triangle=CLEAR candidate");
            telemetry.addLine("Lens center 14 in; back to hive; only selected near-cell tag. All motors untouched.");
            telemetry.update();
            RobotLog.ii("PollenHive","CHECK slot=%d state=%s camera=%s",slot,state,camera==null?error:camera.status);
        }
        buttons();
    }
    private void begin(String label,double now) {
        this.label=label; capture=new IsHiveReadyForShot.Capture(); captureAt=now; detector=null;
        // A recapture cannot accidentally fall back to the previous envelope after failure.
        if(label.equals("UP")) up=null; else down=null;
        message="Hold real hive stationary "+label+" for 5 seconds";
    }
    private void buttons() {
        prevUp=gamepad1.dpad_up; prevDown=gamepad1.dpad_down;
        prevLeft=gamepad1.dpad_left; prevRight=gamepad1.dpad_right;
        prevSquare=gamepad1.square; prevCircle=gamepad1.circle; prevCross=gamepad1.cross; prevTriangle=gamepad1.triangle;
    }
    @Override public void stop() { if(camera!=null) camera.stop(); }
}
