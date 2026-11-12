package org.firstinspires.ftc.teamcode.pollen;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/** Hardware adapter for the two experimental modes. Legacy match autos are unchanged. */
public abstract class PollenHiveOpMode extends OpMode {
    protected abstract boolean secondary();
    private final ElapsedTime clock=new ElapsedTime();
    private final DcMotor[] drive=new DcMotor[4];
    private DcMotorEx launcher;
    private DcMotor intake,agitator;
    private PinpointLocalizer localizer;
    private GoBildaPinpointDriver pinpoint;
    private PollenHiveRoute settings;
    private PollenHiveCamera camera;
    private IsHiveReadyForShot hive;
    private PollenHiveAuto auto;
    private PollenHiveAuto.Output last;
    private String fault,message="",logged="";
    private int row, checkedSlot=-1;
    private String profileStatus="Not checked";
    private boolean up,down,left,right,square,triangle;
    private double displayAt=-1;
    private String settingsName() { return secondary()?"secondary-route":"primary-route"; }
    @Override public final void init() {
        settings=new PollenHiveRoute(secondary());
        try { settings.load(PollenHiveFiles.read(settingsName())); message="Saved settings loaded; execution reset to DRY"; }
        catch(Exception ex) { message="Using CAD defaults: "+ex.getMessage(); }
        try {
            for(int i=0;i<4;i++) {
                drive[i]=hardwareMap.get(DcMotor.class,PollenConfig.DRIVE_NAMES[i]); drive[i].setPower(0);
                drive[i].setDirection(PollenConfig.DRIVE_REVERSED[i]?DcMotorSimple.Direction.REVERSE:DcMotorSimple.Direction.FORWARD);
                drive[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drive[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            launcher=hardwareMap.get(DcMotorEx.class,"pollenLauncher"); launcher.setPower(0);
            launcher.setDirection(DcMotorSimple.Direction.REVERSE);
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(40,0,0,12.5));
            intake=hardwareMap.get(DcMotor.class,"intake"); intake.setPower(0);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            agitator=hardwareMap.get(DcMotor.class,"agitator"); agitator.setPower(0);
            agitator.setDirection(DcMotorSimple.Direction.REVERSE);
            for(DcMotor m:new DcMotor[]{intake,agitator}) {
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            localizer=PollenOdometryHardware.open(hardwareMap);
            pinpoint=PollenOdometryHardware.device(hardwareMap);
        } catch(Exception ex) { fault=ex.toString(); zero(); }
        init_loop();
    }
    @Override public final void init_loop() {
        if(gamepad1.dpad_up&&!up) row=(row+settings.v.length-1)%settings.v.length;
        if(gamepad1.dpad_down&&!down) row=(row+1)%settings.v.length;
        int delta=(gamepad1.dpad_right&&!right?1:0)-(gamepad1.dpad_left&&!left?1:0);
        if(delta!=0) settings.edit(row,delta);
        if(gamepad1.triangle&&!triangle) { settings.defaults(); message="CAD defaults restored; DRY execution"; }
        String validation="READY";
        try { settings.validate(); } catch(Exception ex) { validation="BLOCKED: "+ex.getMessage(); }
        if(gamepad1.square&&!square) {
            try { settings.validate(); PollenHiveFiles.save(settingsName(),settings.properties()); message="Settings saved (execution always reloads DRY)"; }
            catch(Exception ex) { message="SAVE FAILED: "+ex.getMessage(); }
        }
        try { if(localizer!=null) localizer.update(); } catch(Exception ex) { fault=ex.toString(); }
        if(checkedSlot!=(int)settings.v[0]) {
            checkedSlot=(int)settings.v[0];
            try {
                IsHiveReadyForShot.Profile c=PollenHiveFiles.loadProfile(checkedSlot);
                profileStatus="Slot "+checkedSlot+", tag "+c.tag+", pipeline "+c.pipeline+(c.down==null?", UP-only":", UP/DOWN");
            } catch(Exception ex) { profileStatus="MISSING/INVALID: "+ex.getMessage(); }
        }
        telemetry.addData("Profile",profileStatus);
        telemetry.addData("BUILD",PollenConfig.BUILD);
        telemetry.addData("EXPERIMENTAL",secondary()?"Wait for near cell UP, shoot, park":"Shoot, garden/front intake, return, wait/shoot, park");
        telemetry.addData("EXECUTION",settings.v[1]==0?"DRY: all motors zero; virtual launcher speed":settings.v[1]==1?"DRIVE: drivetrain active; intake/launcher/agitator OFF":"LIVE: all mechanisms active after START");
        telemetry.addData("Placement",PollenFacing.placement());
        for(int i=Math.max(0,row-2);i<=Math.min(settings.v.length-1,row+2);i++)
            telemetry.addData((i==row?"> ":"  ")+i+" "+PollenHiveRoute.NAMES[i],"%.2f",settings.v[i]);
        telemetry.addData("Estimated garden / park seconds","%.1f / %.1f",PollenHiveRoute.planned(settings.garden()),PollenHiveRoute.planned(settings.park()));
        telemetry.addData("Odometry",PollenOdometryHardware.status(pinpoint));
        telemetry.addData("Status",fault==null?validation:fault);
        telemetry.addData("Settings",message);
        telemetry.addLine("Dpad up/down row, left/right edit; Square save; Triangle CAD defaults.");
        telemetry.addLine("Coordinates: cm from START, forward=toward hive, left=left looking at hive. Heading deg CCW.");
        telemetry.addLine("DRY/LIVE require a saved tested camera profile; DRIVE can run without camera.");
        telemetry.update();
        up=gamepad1.dpad_up; down=gamepad1.dpad_down; left=gamepad1.dpad_left; right=gamepad1.dpad_right;
        square=gamepad1.square; triangle=gamepad1.triangle;
    }
    @Override public final void start() {
        clock.reset();
        if(fault!=null) return;
        try {
            settings.validate();
            localizer.update();
            if(!"READY".equals(PollenOdometryHardware.status(pinpoint))) throw new IllegalStateException("Pinpoint must be READY at START");
            localizer.setPose(Pose.zero());
            IsHiveReadyForShot.Profile profile=null;
            if(settings.v[1]!=1) {
                profile=PollenHiveFiles.loadProfile((int)settings.v[0]);
                camera=new PollenHiveCamera(hardwareMap,profile.pipeline,profile.tag);
            }
            hive=new IsHiveReadyForShot(profile);
            auto=new PollenHiveAuto(settings,hive); auto.start(clock.seconds());
            RobotLog.ii("PollenHive","START secondary=%s settings=%s",secondary(),settings.properties());
        } catch(Exception ex) { fault=ex.toString(); zero(); }
    }
    @Override public final void loop() {
        double now=clock.seconds();
        try {
            if(fault!=null||auto==null) { zero(); telemetry.addData("BLOCKED",fault); telemetry.update(); return; }
            localizer.update(); Pose p=localizer.pose();
            double[] pose=PollenFacing.routePose(p.x(),p.y(),p.heading());
            boolean ready="READY".equals(PollenOdometryHardware.status(pinpoint));
            IsHiveReadyForShot.Frame frame=camera==null?null:camera.read();
            // DRY predicts feed gating with a virtual flywheel; real motors stay zero.
            double velocity=settings.v[1]==0?(last==null?0:last.launcher):launcher.getVelocity();
            last=auto.tick(now,frame,pose[0],pose[1],pose[2],velocity,ready);
            if(settings.v[1]==0) zero(); else {
                double[] wheels=PollenFacing.wheelPowers(last.forward,last.strafe,last.turn);
                for(int i=0;i<4;i++) drive[i].setPower(wheels[i]);
                launcher.setVelocity(settings.v[1]==2?last.launcher:0);
                intake.setPower(settings.v[1]==2?last.intake:0);
                agitator.setPower(settings.v[1]==2?last.agitator:0);
            }
            String event=last.phase+" / "+hive.state(now)+" / feed="+last.feed;
            if(!event.equals(logged)) { RobotLog.ii("PollenHive","t=%.3f %s reason=%s pose=%.2f,%.2f,%.2f",now,event,last.reason,pose[0],pose[1],pose[2]); logged=event; }
            if(now-displayAt>=.25) {
                displayAt=now;
                telemetry.addData("Phase",last.phase); telemetry.addData("Hive / ready",hive.state(now)+" / "+hive.isHiveReadyForShot(now));
                telemetry.addData("Feed requested / LIVE",last.feed+" / "+(settings.v[1]==2));
                telemetry.addData("Reason",last.reason); telemetry.addData("Camera",camera==null?"Drive-only":camera.status);
                telemetry.addData("Pose fwd / left / heading","%.1f cm / %.1f cm / %.1f deg",pose[0]*2.54,pose[1]*2.54,Math.toDegrees(pose[2]));
                telemetry.addData("Time",now); telemetry.addData("Launcher measured",launcher.getVelocity()); telemetry.update();
            }
        } catch(Exception ex) { fault=ex.toString(); if(auto!=null) auto.stop(fault); zero(); }
    }
    /** Best effort stops every mapped device even if one disconnected device throws. */
    private void zero() {
        for(DcMotor m:drive) if(m!=null) try { m.setPower(0); } catch(RuntimeException ignored) { }
        for(DcMotor m:new DcMotor[]{launcher,intake,agitator}) if(m!=null) try { m.setPower(0); } catch(RuntimeException ignored) { }
    }
    @Override public final void stop() { if(auto!=null) auto.stop("Driver STOP"); zero(); if(camera!=null) camera.stop(); }
}
