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
    /** A frame without our tags (ball in flight, blur) is not evidence; a contradicting frame is. */
    private static void missesVsContradictions() {
        IsHiveReadyForShot.Frame none;
        IsHiveReadyForShot h=new IsHiveReadyForShot(M,34);
        for(int i=1;i<=12;i++) h.update(frame(i,true),i*.02);
        check(h.isHiveReadyForShot(.24),"Stable UP");
        h.update(new IsHiveReadyForShot.Frame(13),.26); check(h.isHiveReadyForShot(.26)&&h.reason.startsWith("Holding UP"),"One missed frame keeps permission: "+h.reason);
        h.update(new IsHiveReadyForShot.Frame(14,tag(42,49.7,25,40,M)),.28); check(h.isHiveReadyForShot(.28),"Only the other hive in view is a miss, not permission from it");
        h.update(frame(15,true),.30); check(h.isHiveReadyForShot(.30),"Sighting after a miss continues");
        for(int i=16;i<=40;i++) { h.update(new IsHiveReadyForShot.Frame(i),.30+(i-15)*.02); if(.30+(i-15)*.02>.50+1e-9) check(!h.isHiveReadyForShot(.30+(i-15)*.02),"Hidden cell loses permission after MISS_HOLD"); }
        check(h.state(.80)==IsHiveReadyForShot.State.UNKNOWN&&h.reason.contains("not in view"),"Long miss is UNKNOWN: "+h.reason);
        // Hold never outlives the last sighting, even if update stops being called.
        h=new IsHiveReadyForShot(M,34);
        for(int i=1;i<=12;i++) h.update(frame(i,true),i*.02);
        h.update(new IsHiveReadyForShot.Frame(13),.40);
        check(h.isHiveReadyForShot(.40)&&!h.isHiveReadyForShot(.4401),"Getter expires MISS_HOLD after the last sighting, not the last empty frame");
        // A contradiction during a hold revokes at once.
        h=new IsHiveReadyForShot(M,34);
        for(int i=1;i<=12;i++) h.update(frame(i,true),i*.02);
        h.update(new IsHiveReadyForShot.Frame(13),.26); h.update(frame(14,42.6,M),.28);
        check(!h.isHiveReadyForShot(.28),"Between-bands frame after a miss revokes");
        h.update(new IsHiveReadyForShot.Frame(15),.30); h.update(frame(16,true),.32);
        check(!h.isHiveReadyForShot(.32),"Re-confirmation needed after a contradiction");
        h=new IsHiveReadyForShot(M,34);
        for(int i=1;i<=12;i++) h.update(frame(i,true),i*.02);
        h.update(new IsHiveReadyForShot.Frame(13),.26); h.update(frame(14,false),.28);
        check(!h.isHiveReadyForShot(.28),"DOWN frame after a miss revokes");
        // Confirmation counts sightings through misses; it still needs CONFIRM_FRAMES and CONFIRM_SECONDS.
        h=new IsHiveReadyForShot(M,34);
        double first=Double.NaN;
        for(int i=1;i<=40;i++) { double t=i*.02; h.update(i%2==0?frame(i,true):new IsHiveReadyForShot.Frame(i),t); if(h.isHiveReadyForShot(t)&&Double.isNaN(first)) first=t; }
        check(Math.abs(first-.20)<1e-9,"Alternate hit/miss: sightings at .04-.16 s; 0.15 s of agreement first holds at .20: "+first);
        h=new IsHiveReadyForShot(M,34);
        for(int i=1;i<=3;i++) h.update(frame(i,true),i*.02);
        for(int i=4;i<=20;i++) h.update(new IsHiveReadyForShot.Frame(i),i*.02);
        h.update(frame(21,true),.42);
        check(!h.isHiveReadyForShot(.42),"Gap longer than MISS_HOLD restarts confirmation");
        // Camera faults still forget at once.
        h=new IsHiveReadyForShot(M,34);
        for(int i=1;i<=12;i++) h.update(frame(i,true),i*.02);
        h.update(null,.26); check(!h.isHiveReadyForShot(.26),"Missing camera result is a fault, not a miss");
        // Side AUTO identification also tolerates misses but not the other cell.
        h=new IsHiveReadyForShot(M,IsHiveReadyForShot.allowedCells(false,0));
        for(int i=1;i<=8;i++) h.update(i%2==0?frame(i,true):new IsHiveReadyForShot.Frame(i),i*.02);
        check(h.cluster()==34,"Identification survives interleaved misses");
        h=new IsHiveReadyForShot(M,IsHiveReadyForShot.allowedCells(false,0));
        for(int i=1;i<=3;i++) h.update(frame(i,true),i*.02);
        h.update(new IsHiveReadyForShot.Frame(4,tag(42,49.7,1,40,M)),.08);
        h.update(frame(5,true),.10);
        check(h.cluster()<0,"Another allowed cell in between restarts identification");
    }
    private static void unsettledReturnParks() {
        PollenHiveRoute c=new PollenHiveRoute(false); c.v[0]=2;
        PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
        PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot();
        PollenHiveAuto.Output o=null; double off=0; boolean secondFeed=false; Set<String> phases=new HashSet<>();
        for(int i=1;i<=1400;i++) {
            double t=i*.02;
            o=a.tick(t,frame(i,true),robot.x+off,robot.y,robot.h,o==null?0:o.launcher,true);
            if(o.phase.equals("SETTLE")) off=1; // odometry/robot ends 1 in off the shot pose
            phases.add(o.phase.split(":")[0]); secondFeed|=o.phase.equals("SECOND")&&o.feed;
            robot.step(o.forward,o.strafe,o.turn,false,.02);
            if(o.finished) break;
        }
        check(phases.contains("SETTLE")&&!secondFeed&&o.finished&&o.reason.equals("Finished departure"),"Unsettled return skips the second shot and still parks: "+o.reason+" "+phases);
    }
    private static PollenHiveCalibration capture(boolean tilt, IsHiveReadyForShot.Mount mount,
            IsHiveReadyForShot.State expected, double actualHeight) {
        PollenHiveCalibration c=new PollenHiveCalibration(tilt,mount,expected,34);
        for(int i=1;i<=60;i++) c.add(frame(i,actualHeight,M),i*.05);
        return c;
    }
    private static void calibration() {
        IsHiveReadyForShot.Mount unset=M.withTilt(Double.NaN);
        PollenHiveCalibration c=capture(true,unset,IsHiveReadyForShot.State.UP,49.7);
        check(c.problem().isEmpty()&&Math.abs(c.tiltDeg()-35)<1e-8,"Known tilt recovered: "+c.problem());
        check(c.frames==60&&c.candidateMount().tiltUpDeg==35,"Accepted independent frames and rounded preview");
        c.add(frame(60,true),3.1); check(c.frames==60,"Repeated result not counted");
        c=new PollenHiveCalibration(true,unset,IsHiveReadyForShot.State.UP,34);
        for(int i=1;i<60;i++) c.add(new IsHiveReadyForShot.Frame(i,new IsHiveReadyForShot.Tag(34,0,0,.1)),i*.05);
        c.add(frame(60,true),3);
        check(c.frames==1&&c.rejected==59&&!c.problem().isEmpty(),"59 impossible solves + one good frame rejected");
        c=new PollenHiveCalibration(true,unset,IsHiveReadyForShot.State.UP,34);
        for(int i=1;i<=60;i++) c.add(frame(i,true),i*.001);
        check(!c.problem().isEmpty(),"Burst under minimum observation span rejected");
        c=new PollenHiveCalibration(true,unset,IsHiveReadyForShot.State.UP,34);
        for(int i=1;i<=60;i++) c.add(frame(i,49.7+(i%2==0?4:-4),M),i*.05);
        check(c.problem().contains("noisy"),"Moving hive rejected");
        check(capture(false,M,IsHiveReadyForShot.State.UP,49.7).problem().isEmpty(),"Physical UP verified with production detector");
        check(capture(false,M,IsHiveReadyForShot.State.DOWN,35.6).problem().isEmpty(),"Physical DOWN verified with production detector");
        check(!capture(false,M,IsHiveReadyForShot.State.DOWN,49.7).problem().isEmpty(),"Incorrect physical label cannot pass");
        c=new PollenHiveCalibration(false,M,IsHiveReadyForShot.State.DOWN,34);
        for(int i=1;i<=60;i++) c.add(new IsHiveReadyForShot.Frame(i,tag(34,30,-2,40,M),tag(35,41.2,2,40,M)),i*.05);
        check(!c.problem().isEmpty(),"Disagreeing tags cannot cancel into a passing mean");
        PollenHiveCalibration wrongTilt=capture(true,unset,IsHiveReadyForShot.State.UP,35.6);
        check(wrongTilt.problem().isEmpty(),"Wrong physical input can produce mathematically valid tilt");
        c=capture(false,wrongTilt.candidateMount(),IsHiveReadyForShot.State.DOWN,35.6);
        check(c.falseUp>0&&!c.problem().isEmpty(),"Independent DOWN check exposes mistaken DOWN-as-UP tilt capture");
        c=new PollenHiveCalibration(false,M,IsHiveReadyForShot.State.DOWN,34);
        for(int i=1;i<=60;i++) c.add(new IsHiveReadyForShot.Frame(i),i*.05);
        check(c.blockedOnly()&&c.problem().isEmpty()&&c.report()[1].startsWith("BLOCKED ONLY"),"Fresh hidden DOWN is blocking evidence, not detected DOWN");
        for(int i=61;i<=80;i++) c.add(new IsHiveReadyForShot.Frame(60),i*.05);
        check(!c.problem().isEmpty(),"Frozen stream after enough empty DOWN frames cannot pass");
        PollenHiveCalibration missing=new PollenHiveCalibration(false,M,IsHiveReadyForShot.State.DOWN,34);
        for(int i=1;i<=60;i++) missing.add(null,i*.05);
        check(!missing.problem().isEmpty(),"Disconnected camera cannot pass hidden-DOWN check");
        PollenHiveCalibrationSession session=new PollenHiveCalibrationSession(unset);
        session.beginTilt(); session.accept(capture(true,unset,IsHiveReadyForShot.State.UP,49.7));
        check(session.candidate&&!session.canSave(),"Tilt alone not validated");
        session.accept(capture(false,session.live,IsHiveReadyForShot.State.UP,49.7));
        check(!session.canSave(),"UP alone not validated");
        session.accept(capture(false,session.live,IsHiveReadyForShot.State.DOWN,35.6));
        check(session.canSave(),"Both physical labels enable saving");
        session.accept(capture(false,session.live,IsHiveReadyForShot.State.DOWN,49.7));
        check(!session.canSave(),"Failed repeat revokes previous pass");
        session.beginTilt();
        check(!session.candidate&&!session.canSave()&&Double.isNaN(session.live.tiltUpDeg),"New capture discards old candidate and validations");
        session.accept(missing);
        check(!session.canSave(),"Rejected capture cannot revive candidate");
        IsHiveReadyForShot h=hive();
        for(int i=1;i<=3;i++) h.update(frame(i,true),i*.02);
        h.update(null,.08); h.update(frame(4,true),.1);
        check(h.cluster()<0,"Identification must restart after missing frame");
        for(int i=5;i<=7;i++) h.update(frame(i,true),.1+(i-4)*.02);
        check(h.cluster()==34,"Four new consecutive frames lock cell");
        h.reset(); check(h.cluster()==34,"Established cell lock survives reset");
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
        check(cam.changedCount()==0&&cam.overrides().isEmpty()&&Double.isFinite(cam.get("tilt")),"Camera starts at defaults (promoted robot calibration)");
        int tilt=cam.index("tilt"); double tiltDefault=cam.def(tilt);
        cam.v[tilt]=Double.NaN; cam.edit(tilt,1); check(cam.v[tilt]==40&&cam.changed(tilt)&&cam.unsaved(tilt),"First press on NOT SET tilt gives 40");
        cam.edit(tilt,1); cam.edit(tilt,1); check(cam.v[tilt]==40.2,"Steps without float noise: "+cam.v[tilt]);
        check(cam.line(tilt).contains("CHANGED (default "+PollenHiveSettings.format("tilt",tiltDefault)+")")&&cam.line(tilt).contains("NOT SAVED"),"Line shows change: "+cam.line(tilt));
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
        r.values.resetExceptExecution(); check(r.v[0]==0,"Route reset preserves DRY motor policy");
        Properties nan=new Properties(); nan.setProperty("lensHeight","NaN");
        check(cam.load(nan).contains("lensHeight")&&Double.isFinite(cam.get("lensHeight")),"Only tilt may load NaN");
        r.values.resetAll(); check(r.values.changedCount()==0&&r.v[0]==2,"Code defaults remain LIVE for match modes");
        for(PollenHiveSettings.Row[] rows:new PollenHiveSettings.Row[][]{PollenHiveSettings.ROUTE,PollenHiveSettings.CAMERA})
            for(PollenHiveSettings.Row row:rows) for(double d:new double[]{row.primary,row.secondary})
                check(Double.isNaN(d)||(d>=row.min&&d<=row.max),"Default inside its range: "+row.key);
    }
    private static void routes() {
        PollenHiveRoute c=new PollenHiveRoute(false); c.validate();
        System.out.printf(Locale.US,"Hive CAD defaults: garden %.2f s, park %.2f s (estimates only)%n",PollenHiveRoute.planned(c.garden()),PollenHiveRoute.planned(c.park()));
        PollenHiveRoute.Node[] nodes=c.garden();
        for(PollenHiveRoute.Node node:nodes)
            check(!node.turn&&node.h==0,"Garden keeps starting heading throughout");
        check(nodes[2].collect&&nodes[2].h==0,"Pickup faces front toward garden wall");
        check(nodes[nodes.length-1].x==0&&nodes[nodes.length-1].y==0&&nodes[nodes.length-1].h==0,"Return restores shoot pose");
        check(Math.abs(PollenFieldGeometry.RED_HIVE_X_INCHES-nodes[2].y+60.585)<1e-6,"Pickup center derived from side-wall clearance");
        Properties oldHeading=new Properties(); oldHeading.setProperty("transitHeading","-90");
        check(c.values.load(oldHeading).contains("transitHeading")&&c.values.get("transitHeading")==0,
                "Old saved transit heading cannot restore turns");
        c.validate();
        c.v[13]=-30; boolean rejected=false; try { c.validate(); } catch(IllegalArgumentException ex) { rejected=true; }
        check(rejected,"Route crossing end wall rejected");
        new PollenHiveRoute(true).validate();
    }
    private static void secondaryGates() {
        PollenHiveRoute c=new PollenHiveRoute(true); c.v[0]=2;
        PollenHiveAuto a=new PollenHiveAuto(c,hive(),PollenHiveAuto.Run.SHOT_ONLY); a.start(0);
        for(int i=1;i<=100;i++) {
            boolean up=i<=50||i>=65;
            check(!a.tick(i*.02,frame(i,up),0,0,0,1180,true).feed,"DOWN resets the one-second UP timer");
        }
        PollenHiveAuto.Output o=null;
        for(int i=101;i<=140;i++) {
            o=a.tick(i*.02,frame(i,true),0,0,0,1100,true);
            check(!o.feed,"Must reach launcher speed to start");
        }
        check(o.decision.contains("hive_up_latched=true"),"Stable UP latches while spinning up");
        o=a.tick(2.82,frame(141,true),0,0,0,1180,true);
        check(o.feed,"Secondary starts once the launcher is ready");
        for(int i=142;i<=343;i++) {
            double now=i*.02;
            o=a.tick(now,i<180?frame(i,false):null,0,0,0,1050,true);
            check(o.feed==!o.finished,"Active volley continues through confirmed DOWN, camera loss and low speed");
            check(o.finished?now-2.82>=4:now-2.82<4.000001,"Volley ends on the first tick after four seconds");
            if(o.finished) break;
        }
        check(o.finished&&o.reason.equals("Stationary shot complete")&&o.decision.contains("requested_s=4.000"),
                "Secondary completes its full four-second volley");

        a=new PollenHiveAuto(c,hive(),PollenHiveAuto.Run.SHOT_ONLY); a.start(0);
        for(int i=1;i<=100;i++) o=a.tick(i*.02,frame(i,true),0,0,0,1180,true);
        check(o.feed,"Feeding before simulated stall");
        o=a.tick(2.4,frame(101,true),0,0,0,1180,true);
        check(o.finished&&o.launcher==0&&o.intake==0,"Loop stall latches zero");
        check(a.tick(2.42,frame(102,true),0,0,0,1180,true).finished,"Terminal stays stopped");

        a=new PollenHiveAuto(c,hive(),PollenHiveAuto.Run.SHOT_ONLY); a.start(0);
        for(int i=1;i<=1180;i++) {
            o=a.tick(i*.02,frame(i,i>1100),0,0,0,1180,true);
            check(!o.feed,"Skip a late volley that cannot finish");
            if(o.finished) break;
        }
        check(o.finished&&o.reason.contains("Insufficient time for full volley"),"Preserve the full-volley time reserve");
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
        PollenHiveRoute c=new PollenHiveRoute(false); c.v[0]=2; c.v[5]=10;
        PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
        PollenHiveAuto.Output o=null;
        boolean fed=false,skipped=false;
        for(int i=1;i<=700;i++) {
            double t=i*.02;
            double velocity=t<8.0?0:(o!=null&&o.launcher>0?1180:0);
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
    private static void practice() {
        for(boolean secondary:new boolean[]{false,true}) {
            PollenHiveRoute c=new PollenHiveRoute(secondary); c.v[0]=2;
            PollenHiveAuto a=new PollenHiveAuto(c,hive(),PollenHiveAuto.Run.SHOT_ONLY); a.start(0);
            PollenHiveAuto.Output o=null; boolean fed=false;
            for(int i=1;i<=1351;i++) {
                double t=i*.02;
                o=a.tick(t,frame(i,t>1),0,0,0,1180,true);
                check(o.forward==0&&o.strafe==0&&o.turn==0,"Shot-only never drives");
                check(!o.phase.startsWith("PARK")&&!o.phase.startsWith("GARDEN"),"Shot-only never enters route");
                if(t<=1) check(!o.feed,"Shot-only waits for physical UP");
                fed|=o.feed; if(o.finished) break;
            }
            check(fed&&o.finished&&o.reason.equals("Stationary shot complete"),"Stationary volley completes without departure");
        }
        for(int execution:new int[]{1,2}) {
            PollenHiveRoute c=new PollenHiveRoute(false); c.v[0]=execution;
            PollenHiveAuto a=new PollenHiveAuto(c,hive(),PollenHiveAuto.Run.GARDEN_RETURN); a.start(0);
            PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot();
            PollenHiveAuto.Output o=null; boolean collected=false;
            for(int i=1;i<=1351;i++) {
                o=a.tick(i*.02,null,robot.x,robot.y,robot.h,0,true);
                check(o.launcher==0&&o.agitator==0&&!o.feed,"Garden practice never fires");
                check(!o.phase.startsWith("FIRST")&&!o.phase.startsWith("SECOND")&&!o.phase.startsWith("PARK"),"Garden practice skips shots and departure");
                collected|=o.intake>0;
                robot.step(o.forward,o.strafe,o.turn,false,.02);
                if(o.finished) break;
            }
            check(o.finished&&o.reason.equals("Garden returned and settled at S"),"Garden-only returns: "+o.reason);
            check(Math.hypot(robot.x,robot.y)<=.5&&Math.abs(robot.h)<=Math.toRadians(2),"Garden ends settled at original S");
            check(collected==(execution==2),"Only LIVE garden test runs collection intake");
        }
    }
    private static void noCoastGate() {
        for(double velocity:new double[]{0,1180,-1180,2500}) {
            for(PollenHiveAuto.Run run:new PollenHiveAuto.Run[]{PollenHiveAuto.Run.MATCH,PollenHiveAuto.Run.GARDEN_RETURN}) {
                PollenHiveRoute c=new PollenHiveRoute(false); c.v[0]=1;
                PollenHiveAuto a=new PollenHiveAuto(c,hive(),run); a.start(0);
                PollenHiveAuto.Output o=a.tick(.02,null,0,0,0,velocity,true);
                if(run==PollenHiveAuto.Run.MATCH) o=a.tick(.04,null,0,0,0,velocity,true);
                check(o.phase.startsWith("GARDEN"),"No flywheel speed or minimum-time gate before garden");
                check(o.launcher==0&&o.agitator==0&&o.intake==0,"Departure transition switches all mechanisms off");
            }
        }
        PollenHiveRoute c=new PollenHiveRoute(false); c.v[0]=2;
        PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
        PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot();
        boolean fed=false,collected=false; String previous="";
        for(int i=1;i<=1400;i++) {
            // Hold measured flywheel speed high for the WHOLE run, even with its command off.
            PollenHiveAuto.Output o=a.tick(i*.02,frame(i,true),robot.x,robot.y,robot.h,1180,true);
            fed|=o.feed;
            if(o.phase.startsWith("GARDEN")) {
                check(o.launcher==0&&o.agitator==0&&!o.feed,"Garden never powers launcher or agitator, even while collecting");
                collected|=o.intake>0;
                if(previous.equals("FIRST")) check(o.forward==0&&o.strafe==0&&o.turn==0&&o.intake==0,
                        "Shot exit has a zero-output tick before path motion");
            }
            previous=o.phase;
            robot.step(o.forward,o.strafe,o.turn,false,.02);
            if(o.finished) break;
        }
        check(fed&&collected,"A live first shot leads to collection despite persistent high measured flywheel speed");
    }
    private static PollenHiveAuto.Output oDummy;
    /** Play 2 secondary: shoot, wait in the corner out of the hero's way, park with 2 s left. */
    private static void makeRoom() {
        boolean rejected=false;
        try { new PollenHiveRoute(false,true); } catch(IllegalArgumentException ex) { rejected=true; }
        check(rejected,"Make room is a secondary mode only");
        PollenHiveRoute c=new PollenHiveRoute(true,true); c.validate();
        PollenHiveRoute.Node[] room=c.corner(); PollenHiveRoute.Node k=room[room.length-1];
        double fieldX=PollenFieldGeometry.SUPPORT_START_X_INCHES+k.y, fieldZ=PollenFieldGeometry.supportStartZInches()+k.x;
        double corner=-PollenFieldGeometry.END_WALL_INCHES+PollenFieldGeometry.ROBOT_HALF_INCHES+1;
        check(Math.abs(fieldX-corner)<1e-3&&Math.abs(fieldZ-corner)<1e-3,"Corner is 1 in off the end and side walls: "+fieldX+","+fieldZ);
        PollenHiveRoute.Node[] park=c.cornerPark(), normal=c.park();
        check(park.length==1&&park[0].x==normal[normal.length-1].x&&park[0].y==normal[normal.length-1].y,"Parks in the same outer slot");
        System.out.printf(Locale.US,"Hive make room: corner %.2f s, corner to park %.2f s (estimates only)%n",
                PollenHiveRoute.planned(room),PollenHiveRoute.planned(park,k.x,k.y));
        // UP from the start: shoot, corner, wait, park when 2 s are left of 27.
        for(boolean tips:new boolean[]{true,false}) {
            PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
            PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot();
            PollenHiveAuto.Output o=null; List<String> order=new ArrayList<>();
            double roomAt=Double.NaN,waitAt=Double.NaN,parkAt=Double.NaN,t=0; boolean fed=false;
            for(int i=1;i<=1400;i++) {
                t=i*.02;
                o=a.tick(t,frame(i,tips),robot.x,robot.y,robot.h,o==null?0:o.launcher,true);
                String ph=o.phase.split(":")[0];
                if(order.isEmpty()||!order.get(order.size()-1).equals(ph)) order.add(ph);
                fed|=o.feed;
                if(ph.equals("ROOM")&&Double.isNaN(roomAt)) roomAt=t;
                if(ph.equals("WAIT")&&Double.isNaN(waitAt)) waitAt=t;
                if(ph.equals("PARK")&&Double.isNaN(parkAt)) parkAt=t;
                if(ph.equals("ROOM")||ph.equals("WAIT")||ph.equals("PARK")) check(o.launcher==0&&o.agitator==0&&!o.feed,"No shooting after leaving");
                if(ph.equals("WAIT")) check(o.forward==0&&o.strafe==0&&o.turn==0,"Sits still in the corner");
                robot.step(o.forward,o.strafe,o.turn,false,.02);
                if(ph.equals("WAIT")) check(Math.hypot(robot.x-k.x,robot.y-k.y)<=.75,"Waits at the corner");
                if(o.finished) break;
            }
            System.out.printf(Locale.US,"Hive make room tips=%s: room %.2f s, corner %.2f s, park %.2f s, done %.2f s %s%n",tips,roomAt,waitAt,parkAt,t,order);
            check(fed==tips,"Shoots only when UP");
            check(order.subList(order.size()-4,order.size()).equals(Arrays.asList("ROOM","WAIT","PARK","DONE")),"Corner, wait, then park: "+order);
            check(parkAt>=25-1e-9&&parkAt<25.1,"Park starts when 2 s are left: "+parkAt);
            check(o.finished&&o.reason.equals("Finished departure")&&t<27,"Parks before the 27 s stop: "+o.reason+" at "+t);
            check(Math.hypot(robot.x-park[0].x,robot.y-park[0].y)<=.75,"Ends in the outer slot");
            if(tips) check(roomAt<10,"Heads to the corner right after the shot: "+roomAt);
        }
        c.v[PollenHiveRoute.ROOM_PARK_SECONDS_LEFT]=.5; rejected=false;
        try { c.validate(); } catch(IllegalArgumentException ex) { rejected=true; }
        check(rejected,"Park time shorter than the corner-to-park drive is rejected at INIT");
        c.v[PollenHiveRoute.ROOM_PARK_SECONDS_LEFT]=2; c.v[PollenHiveRoute.ROOM_LEFT]=-140; rejected=false;
        try { c.validate(); } catch(IllegalArgumentException ex) { rejected=true; }
        check(rejected,"Corner past the side wall is rejected at INIT");
    }
    private static void matchAllianceSetup() {
        for (int play = 0; play < 3; play++) {
            boolean secondary = play != 0;
            PollenHiveRoute route = new PollenHiveRoute(secondary, play == 2);
            Properties oldProfile = new Properties();
            oldProfile.setProperty("side", "2");
            route.values.load(oldProfile);
            check(route.side() == 2, "Calibration can still load a saved side");
            for (int color = 1; color <= 2; color++) {
                route.beginMatchSetup();
                check(route.needsMatchAlliance(), "New INIT ignores old/saved alliance");
                boolean blocked = false;
                try { route.validateForStart(); }
                catch (IllegalArgumentException ex) { blocked = ex.getMessage().contains("Choose alliance"); }
                check(blocked, "Match START requires a choice");
                route.selectMatchAlliance(color == 2);
                route.validateForStart();
                int[] cells = IsHiveReadyForShot.allowedCells(secondary, route.side());
                int expected = secondary ? (color == 1 ? 30 : 38) : (color == 1 ? 34 : 42);
                check(cells.length == 1 && cells[0] == expected, "Only the correct role and color cell is eligible");
                check(route.values.unsavedCount() == 0 && route.values.get("side") == 2,
                        "A match choice never overwrites calibration side");
                check(route.toString().contains("side=" + color + ".0"), "Log the effective alliance");
            }
        }
    }

    public static void main(String[] args) {
        matchAllianceSetup();
        classifier(); missesVsContradictions(); unsettledReturnParks(); calibration(); sides(); settings(); routes(); secondaryGates(); primaryCycle(false,false); primaryCycle(false,true); primaryCycle(true,false); slowLauncherBudget(); motionFaults(); practice(); noCoastGate(); makeRoom();
        System.out.println("PASS PollenHiveTest: "+checks+" checks");
    }
}
