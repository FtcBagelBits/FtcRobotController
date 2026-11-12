package org.firstinspires.ftc.teamcode.pollen;

import java.util.*;

/** Computer-only regression tests. Synthetic observations and a simple drivetrain are NOT field validation. */
public final class PollenHiveTest {
    private static int checks;
    private static void check(boolean ok,String why) { checks++; if(!ok) throw new AssertionError(why); }
    /** Test mount: lens 14 in, camera tilted up 35 deg, centered. */
    static final IsHiveReadyForShot.Mount M=new IsHiveReadyForShot.Mount(14,35,0,49.7,35.6,3,12.5);
    /** Camera-space tag for a tag at this height (in), this far to the side (in, + = camera right)
     * and this far away horizontally (in), seen through mount m. Inverse of Mount.heightIn. */
    static IsHiveReadyForShot.Tag tag(int id,double heightIn,double sideIn,double distIn,IsHiveReadyForShot.Mount m) {
        double a=Math.toRadians(m.tiltUpDeg),rise=(heightIn-m.lensHeightIn)*.0254,d=distIn*.0254;
        return new IsHiveReadyForShot.Tag(id,(sideIn-m.sidewaysCorrectionIn)*.0254,d*Math.sin(a)-rise*Math.cos(a),d*Math.cos(a)+rise*Math.sin(a));
    }
    /** Red primary view: our cell 34-37 straight behind, plus one blue tag 25 in to the side. */
    private static IsHiveReadyForShot.Frame frame(double stamp,boolean up) { return frame(stamp,up?49.7:35.6,M); }
    private static IsHiveReadyForShot.Frame frame(double stamp,double h,IsHiveReadyForShot.Mount m) {
        return new IsHiveReadyForShot.Frame(stamp,tag(34,h,-6.9,40,m),tag(35,h,-2.3,40,m),tag(36,h,2.3,40,m),
            tag(37,h,6.9,40,m),tag(42,49.7,25,40,m));
    }
    private static IsHiveReadyForShot hive() { return new IsHiveReadyForShot(M); }
    private static void classifier() {
        check(IsHiveReadyForShot.clusterOf(30)==30&&IsHiveReadyForShot.clusterOf(37)==34&&IsHiveReadyForShot.clusterOf(45)==42
            &&IsHiveReadyForShot.clusterOf(29)<0&&IsHiveReadyForShot.clusterOf(46)<0,"Tag IDs map to cells");
        IsHiveReadyForShot.Tag t=tag(35,49.7,-2,40,M);
        check(Math.abs(M.heightIn(t)-49.7)<1e-9,"Height formula inverts");
        check(Math.abs(IsHiveReadyForShot.solveTiltDeg(t,49.7,14)-35)<1e-9,"Tilt solve inverts");
        for(double d:new double[]{25,40,60}) check(Math.abs(M.heightIn(tag(35,49.7,0,d,M))-49.7)<1e-9,"Height does not depend on distance");
        check(M.classify(51)==IsHiveReadyForShot.State.UP&&M.classify(37)==IsHiveReadyForShot.State.DOWN
            &&M.classify(42.6)==IsHiveReadyForShot.State.UNKNOWN,"Bands: UP, DOWN, between = UNKNOWN");
        IsHiveReadyForShot h=hive();
        check(!h.isHiveReadyForShot(0),"Starts unknown");
        for(int i=1;i<=12;i++) h.update(frame(i,true),i*.02);
        check(h.cluster()==34,"Locks the cell straight behind (red audience), ignoring the other hive");
        check(h.isHiveReadyForShot(.24),"Stable UP grants permission");
        check(!h.isHiveReadyForShot(.45),"Getter expires without more update calls");
        h.update(frame(12,true),.46); check(!h.isHiveReadyForShot(.46),"Frozen camera cannot rearm");
        for(int i=13;i<=26;i++) h.update(frame(i,true),.46+(i-12)*.02);
        check(h.isHiveReadyForShot(.74),"Distinct frames recover");
        h.update(frame(27,false),.76); check(!h.isHiveReadyForShot(.76),"One contradiction revokes immediately");
        for(int i=28;i<=40;i++) h.update(frame(i,false),.76+(i-27)*.02);
        check(h.state(1.02)==IsHiveReadyForShot.State.DOWN,"DOWN confirms independently");
        h.update(null,1.04); check(h.state(1.04)==IsHiveReadyForShot.State.UNKNOWN,"Missing frame is UNKNOWN");
        h.update(frame(1,true),1.06); check(!h.isHiveReadyForShot(1.06),"Camera reboot requires reconfirmation");
        h.reset(); check(h.cluster()==34,"Reset keeps which cell is ours");
        // Only the other alliance's UP cell in view: never counts for us.
        for(int i=2;i<30;i++) h.update(new IsHiveReadyForShot.Frame(i,tag(42,49.7,25,40,M),tag(43,49.7,20,40,M)),1.1+i*.02);
        check(!h.isHiveReadyForShot(1.7),"Other hive's UP cell never permits a shot");
        // Our cell half way through a tip, and tags that disagree.
        for(int i=30;i<50;i++) h.update(frame(i,42.6,M),1.7+(i-30)*.02);
        check(!h.isHiveReadyForShot(2.1),"Tipping (between bands) is UNKNOWN");
        for(int i=50;i<70;i++) h.update(new IsHiveReadyForShot.Frame(i,tag(34,49.7,-6.9,40,M),tag(35,35.6,-2.3,40,M)),2.1+(i-50)*.02);
        check(!h.isHiveReadyForShot(2.5),"Tags that disagree are UNKNOWN");
        for(int i=70;i<90;i++) h.update(new IsHiveReadyForShot.Frame(i,tag(36,49.7,2.3,40,M)),2.5+(i-70)*.02);
        check(h.isHiveReadyForShot(2.88),"One of our tags is enough");
        h.update(new IsHiveReadyForShot.Frame(91,new IsHiveReadyForShot.Tag(36,.05,-.5,0)),2.9);
        check(!h.isHiveReadyForShot(2.9),"Impossible pose rejected");
        // Never seen our cell: other hive alone does not get locked as ours.
        h=hive();
        for(int i=1;i<30;i++) h.update(new IsHiveReadyForShot.Frame(i,tag(42,49.7,16,40,M)),i*.02);
        check(h.cluster()<0&&!h.isHiveReadyForShot(.6),"Tags 16 in to the side are not our cell");
        // Secondary: our cell first seen when it comes UP; the flip side of a cell may be in view first.
        h=hive();
        for(int i=1;i<30;i++) h.update(new IsHiveReadyForShot.Frame(i),i*.02);
        check(h.cluster()<0&&!h.isHiveReadyForShot(.6),"No tags: no cell, no shot");
        for(int i=30;i<45;i++) h.update(new IsHiveReadyForShot.Frame(i,tag(30,49.7,-9.6,40,M),tag(31,49.7,-5,40,M)),i*.02);
        check(h.cluster()==30&&h.isHiveReadyForShot(.88),"Secondary locks red scoring cell when it comes UP");
        // Not calibrated: identifies the cell, never shoots.
        h=new IsHiveReadyForShot(M.withTilt(Double.NaN));
        for(int i=1;i<=30;i++) h.update(frame(i,true),i*.02);
        check(h.cluster()==34&&!h.isHiveReadyForShot(.6)&&h.reason.contains("not calibrated"),"Uncalibrated tilt never shoots");
        // Off-center camera: corrected by the sideways number.
        IsHiveReadyForShot.Mount off=new IsHiveReadyForShot.Mount(14,35,-4,49.7,35.6,3,12.5);
        check(IsHiveReadyForShot.nearestCluster(frame(1,49.7,off),off)==34,"Sideways correction keeps our cell");
    }
    private static void calibration() {
        // Truth: tilt 33.3 deg, camera 1.5 in off center. Code: tilt not set, sideways 0.
        IsHiveReadyForShot.Mount truth=new IsHiveReadyForShot.Mount(14,33.3,1.5,49.7,35.6,3,12.5);
        IsHiveReadyForShot.Mount code=new IsHiveReadyForShot.Mount(14,Double.NaN,0,49.7,35.6,3,12.5);
        PollenHiveCalibration c=new PollenHiveCalibration(true,code);
        for(int i=1;i<=10;i++) c.add(frame(i,49.7,truth));
        check(!c.problem().isEmpty(),"Too few frames rejected");
        for(int i=11;i<=60;i++) { c.add(frame(i,49.7+.2*Math.sin(i),truth)); c.add(frame(i,49.7,truth)); }
        check(c.frames==60,"Duplicate frames ignored");
        check(c.problem().isEmpty(),"Good tilt capture accepted: "+c.problem());
        check(Math.abs(c.tiltDeg()-33.3)<.05,"Tilt recovered: "+c.tiltDeg());
        check(Math.abs(c.sidewaysCorrectionIn()-1.5)<1e-6,"Sideways correction recovered: "+c.sidewaysCorrectionIn());
        check(c.report()[1].equals("Camera tilt up deg = 33.3")&&c.report()[2].equals("Camera sideways correction in = 1.50"),"Result lines: "+String.join(" | ",c.report()));
        c=new PollenHiveCalibration(true,code);
        for(int i=1;i<=60;i++) c.add(frame(i,49.7+(i%2==0?4:-4),truth));
        check(c.problem().contains("noisy"),"Moving hive rejected");
        // Height checks with the found calibration.
        IsHiveReadyForShot.Mount found=truth;
        c=new PollenHiveCalibration(false,found);
        for(int i=1;i<=40;i++) c.add(frame(i,49.9,truth));
        check(c.report()[1].startsWith("PASS")&&Double.isNaN(c.suggestedDownIn()),"UP height check passes, nothing to save");
        c=new PollenHiveCalibration(false,found);
        for(int i=1;i<=40;i++) c.add(frame(i,37.2,truth));
        check(c.report()[1].contains("DOWN tag height in = 37.2")&&c.suggestedDownIn()==37.2,"DOWN off by 1.6 in suggests the update: "+c.report()[1]);
        c=new PollenHiveCalibration(false,found);
        for(int i=1;i<=40;i++) c.add(frame(i,42.6,truth));
        check(c.report()[1].startsWith("BETWEEN"),"Between bands reported");
    }
    private static void sides() {
        check(Arrays.equals(IsHiveReadyForShot.allowedCells(false,0),new int[]{34,42})&&Arrays.equals(IsHiveReadyForShot.allowedCells(true,0),new int[]{30,38})
            &&Arrays.equals(IsHiveReadyForShot.allowedCells(false,1),new int[]{34})&&Arrays.equals(IsHiveReadyForShot.allowedCells(true,2),new int[]{38}),
            "Primary = 34 red / 42 blue, secondary = 30 red / 38 blue");
        // AUTO primary: a centered secondary-side cell (30) is not a candidate; ours (34) is.
        IsHiveReadyForShot h=new IsHiveReadyForShot(M,IsHiveReadyForShot.allowedCells(false,0));
        for(int i=1;i<20;i++) h.update(new IsHiveReadyForShot.Frame(i,tag(30,49.7,0,40,M)),i*.02);
        check(h.cluster()<0&&!h.isHiveReadyForShot(.4),"AUTO primary ignores secondary-side cells");
        for(int i=20;i<40;i++) h.update(frame(i,true),i*.02);
        check(h.cluster()==34&&h.isHiveReadyForShot(.78),"AUTO primary finds red 34-37");
        // Forced BLUE primary: cell 42 from the start, red tags straight behind never count.
        h=new IsHiveReadyForShot(M,IsHiveReadyForShot.allowedCells(false,2));
        check(h.cluster()==42,"Forced side fixes the cell before any frame");
        for(int i=1;i<20;i++) h.update(frame(i,true),i*.02);
        check(!h.isHiveReadyForShot(.38)&&h.lastTagCount==0,"Forced BLUE on a red start: red tags ignored, far blue tag (25 in aside) ignored");
        for(int i=20;i<40;i++) h.update(new IsHiveReadyForShot.Frame(i,tag(42,49.7,-3,40,M),tag(43,49.7,2,40,M)),i*.02);
        check(h.isHiveReadyForShot(.78),"Forced BLUE: blue cell UP shoots");
        PollenHiveRoute r=new PollenHiveRoute(false); check(r.side()==0,"Side default is AUTO");
    }
    private static void settings() {
        PollenHiveSettings.Values cam=new PollenHiveSettings.Values(PollenHiveSettings.CAMERA,false);
        check(cam.changedCount()==0&&cam.overrides().isEmpty()&&Double.isNaN(cam.get("tilt")),"Camera starts at defaults, tilt NOT SET");
        int tilt=cam.index("tilt"); cam.edit(tilt,1); check(cam.v[tilt]==40&&cam.changed(tilt)&&cam.unsaved(tilt),"First press on NOT SET tilt gives 40");
        cam.edit(tilt,1); cam.edit(tilt,1); check(cam.v[tilt]==40.2,"Steps without float noise: "+cam.v[tilt]);
        check(cam.line(tilt).contains("CHANGED (default NOT SET)")&&cam.line(tilt).contains("NOT SAVED"),"Line shows change: "+cam.line(tilt));
        cam.markSaved(); check(!cam.unsaved(tilt)&&!cam.line(tilt).contains("NOT SAVED"),"Saved edit no longer NOT SAVED");
        check(cam.overrides().size()==1&&cam.overrides().getProperty("tilt").equals("40.2"),"Only the change is saved");
        PollenHiveSettings.Values back=new PollenHiveSettings.Values(PollenHiveSettings.CAMERA,false);
        check(back.load(cam.overrides()).isEmpty()&&back.get("tilt")==40.2&&back.changedCount()==1&&back.unsavedCount()==0,"Saved change reloads");
        Properties bad=new Properties(); bad.setProperty("nonsense","3"); bad.setProperty("band","99"); bad.setProperty("lensHeight","abc");
        back=new PollenHiveSettings.Values(PollenHiveSettings.CAMERA,false);
        check(back.load(bad).split(", ").length==3&&back.changedCount()==0,"Unknown, out-of-range and bad values ignored and reported");
        PollenHiveRoute r=new PollenHiveRoute(true);
        check(r.v==r.values.v&&r.values.def(22)==20.32&&new PollenHiveRoute(false).values.def(22)==10,"Primary and secondary defaults differ where they should");
        r.v[0]=0; r.v[6]=1200; Properties o=r.values.overrides();
        check(o.size()==1&&o.getProperty("launcher").equals("1200.0"),"Execution is never saved");
        check(r.values.line(0).contains("this run only")&&r.values.changedSummary().equals("Execution=DRY (no motors), Launcher ticks/s=1200"),"Summary: "+r.values.changedSummary());
        r.values.reset(6); check(!r.values.changed(6),"Row reset");
        r.values.resetAll(); check(r.values.changedCount()==0&&r.v[0]==2,"All reset; execution back to LIVE");
        for(PollenHiveSettings.Row[] rows:new PollenHiveSettings.Row[][]{PollenHiveSettings.ROUTE,PollenHiveSettings.CAMERA})
            for(PollenHiveSettings.Row row:rows) for(double d:new double[]{row.primary,row.secondary})
                check(Double.isNaN(d)||(d>=row.min&&d<=row.max),"Default inside its range: "+row.key);
    }
    private static void routes() {
        PollenHiveRoute c=new PollenHiveRoute(false); c.validate();
        System.out.printf(Locale.US,"Hive CAD defaults: garden %.2f s, park %.2f s (estimates only)%n",PollenHiveRoute.planned(c.garden()),PollenHiveRoute.planned(c.park()));
        PollenHiveRoute.Node[] nodes=c.garden();
        check(Math.abs(nodes[3].h+Math.PI/2)<1e-9,"Transit turns front toward garden");
        check(nodes[6].collect&&nodes[6].h==0,"Pickup faces front toward garden wall");
        check(nodes[nodes.length-1].x==0&&nodes[nodes.length-1].y==0&&nodes[nodes.length-1].h==0,"Return restores shoot pose");
        check(Math.abs(PollenFieldGeometry.RED_HIVE_X_INCHES-nodes[6].y+60.585)<1e-6,"Pickup center derived from side-wall clearance");
        c.v[13]=0; boolean rejected=false; try { c.validate(); } catch(IllegalArgumentException ex) { rejected=true; }
        check(rejected,"Turn with no end-wall clearance rejected");
        new PollenHiveRoute(true).validate();
    }
    private static void secondaryGates() {
        PollenHiveRoute c=new PollenHiveRoute(true); c.v[0]=2; c.v[3]=10; c.v[21]=0;
        IsHiveReadyForShot h=hive(); PollenHiveAuto a=new PollenHiveAuto(c,h); a.start(0);
        PollenHiveAuto.Output o=null;
        for(int i=1;i<=150;i++) { o=a.tick(i*.02,frame(i,false),0,0,0,1180,true); check(!o.feed,"Secondary waits DOWN"); }
        for(int i=151;i<=180;i++) o=a.tick(i*.02,frame(i,true),0,0,0,1180,true);
        check(o.feed,"Secondary shoots after confirmed UP");
        o=a.tick(3.62,frame(181,true),0,0,0,0,true); check(!o.feed,"No speed timer bypass");
        o=a.tick(3.64,null,0,0,0,1180,true); check(!o.feed,"Loss revokes feed this loop");
        for(int i=183;i<=210;i++) o=a.tick(i*.02,frame(i,true),0,0,0,1180,true);
        check(o.feed,"Vision recovers");
        o=a.tick(4.22,frame(211,true),1,0,0,1180,true); check(!o.feed,"Moving off shooting pose revokes feed");
        o=a.tick(4.6,frame(212,true),0,0,0,1180,true);
        check(o.finished&&o.launcher==0&&o.intake==0,"Loop stall latches zero");
        check(a.tick(4.62,frame(213,true),0,0,0,1180,true).finished,"Terminal stays stopped");
    }
    private static void primaryCycle(boolean driveOnly,boolean tipped) {
        PollenHiveRoute c=new PollenHiveRoute(false); c.v[0]=driveOnly?1:2;
        IsHiveReadyForShot h=hive(); PollenHiveAuto a=new PollenHiveAuto(c,h); a.start(0);
        PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot();
        Set<String> phases=new HashSet<>(); boolean collect=false,secondFeed=false,firstFeed=false;
        PollenHiveAuto.Output o=null;
        for(int i=1;i<=1400;i++) {
            double t=i*.02;
            double velocity=o==null?0:o.launcher;
            boolean up=!(tipped&&phases.contains("GARDEN"));
            o=a.tick(t,frame(i,up),robot.x,robot.y,robot.h,velocity,true);
            phases.add(o.phase.split(":")[0]);
            if(o.phase.startsWith("GARDEN")&&o.intake>0) collect=true;
            if(o.phase.startsWith("GARDEN")||o.phase.startsWith("PARK")) check(o.launcher==0&&o.agitator==0,"No shooting during motion");
            if(o.phase.equals("FIRST")&&o.feed) firstFeed=true;
            if(o.phase.equals("SECOND")&&o.feed) secondFeed=true;
            if(o.feed) check(Math.hypot(robot.x,robot.y)<=.5&&Math.abs(robot.h)<=Math.toRadians(2),"Feed only at restored shot pose");
            robot.step(o.forward,o.strafe,o.turn,false,.02);
            if(o.finished) { System.out.printf(Locale.US,"Hive simulated %s tipped=%s: %.2fs %s phases=%s%n",driveOnly?"DRIVE":"LIVE",tipped,t,o.reason,phases); break; }
        }
        check(phases.contains("GARDEN"),"Primary must exercise default garden; last="+o.reason);
        check(phases.contains("SECOND"),"Primary must return for second shot; last="+o.reason+" pose="+robot.x+","+robot.y+","+robot.h);
        check(o.finished&&o.reason.equals("Finished departure"),"Default cycle finishes park: "+o.reason);
        if(driveOnly) check(!firstFeed&&!secondFeed&&!collect,"Drive-only disables all mechanism actions");
        else { check(firstFeed&&collect,"Primary fires and collects"); check(secondFeed!=tipped,"Second shot respects current hive state"); }
    }
    private static void slowLauncherBudget() {
        PollenHiveRoute c=new PollenHiveRoute(false); c.v[0]=2;
        PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
        PollenHiveAuto.Output o=null;
        boolean fed=false,skipped=false;
        for(int i=1;i<=400;i++) {
            double t=i*.02;
            double velocity=t<3.1?0:(o!=null&&o.launcher>0?1180:0);
            o=a.tick(t,frame(i,true),0,0,0,velocity,true);
            fed|=o.feed;
            if(o.phase.startsWith("PARK")) { skipped=o.reason.contains("not enough estimated cycle time"); break; }
        }
        check(fed&&skipped,"Slow first shot skips a garden cycle that no longer fits");
        a=new PollenHiveAuto(c,hive()); a.start(0);
        o=a.tick(.02,frame(1,true),0,0,0,1180,false);
        check(o.finished&&o.launcher==0&&o.intake==0,"Pinpoint not READY latches a stop");
        double[] actual=PollenFacing.wheelPowers(.3,.1,.2);
        double[] expected=PollenSequence.wheelPowers(-.3,-.1,.2);
        for(int i=0;i<4;i++) check(actual[i]==expected[i],"Rear-facing conversion matches legacy boundary");
    }
    private static void motionFaults() {
        PollenHiveMotion m=new PollenHiveMotion(new PollenHiveRoute(false).garden());
        PollenHiveMotion.Output o=null;
        for(int i=1;i<=50;i++) o=m.tick(i*.02,0,0,0);
        check(o.fault!=null,"Stuck motion stops");
        PollenHiveRoute c=new PollenHiveRoute(true); c.v[21]=0;
        PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
        for(int i=1;i<=1351;i++) oDummy=a.tick(i*.02,null,0,0,0,0,true);
        check(oDummy.finished,"Deadline stops even with no observation");
    }
    private static PollenHiveAuto.Output oDummy;
    public static void main(String[] args) {
        classifier(); calibration(); sides(); settings(); routes(); secondaryGates(); primaryCycle(false,false); primaryCycle(false,true); primaryCycle(true,false); slowLauncherBudget(); motionFaults();
        System.out.println("PASS PollenHiveTest: "+checks+" checks");
    }
}
