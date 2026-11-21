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
    private static void autonomousTagGate() {
        // M: UP 49.7 +/- 3, so the autonomous UP window is 46.7 to 55.7 in.
        IsHiveReadyForShot h=new IsHiveReadyForShot(M,38);
        for(int i=1;i<=12;i++) {
            h.updateForAutonomous(new IsHiveReadyForShot.Frame(i,tag(38,51.5,14,60,M),tag(39,52,0,60,M),
                    tag(41,60+i*.3,10,60,M)),i*.02);
        }
        check(h.isHiveReadyForShot(.24),"Median ignores one outlier tag; a sideways tag still counts");

        h=new IsHiveReadyForShot(M,38);
        for(int i=1;i<=12;i++) h.updateForAutonomous(new IsHiveReadyForShot.Frame(i,tag(41,54.1+(i%2)*.4,14,60,M)),i*.02);
        check(h.isHiveReadyForShot(.24),"A high UP reading (54-55 in, seen on the field) qualifies");
        h.updateForAutonomous(new IsHiveReadyForShot.Frame(13,tag(41,58,14,60,M)),.26);
        h.updateForAutonomous(new IsHiveReadyForShot.Frame(14,tag(41,40,14,60,M)),.28);
        check(h.isHiveReadyForShot(.28),"Two out-of-window frames in a row do not revoke permission");
        h.updateForAutonomous(new IsHiveReadyForShot.Frame(15,tag(41,54.1,14,60,M)),.30);
        check(h.isHiveReadyForShot(.30),"An in-window frame after forgiven frames keeps permission");
        for(int i=16;i<=18;i++) h.updateForAutonomous(new IsHiveReadyForShot.Frame(i,tag(41,42,14,60,M)),i*.02);
        check(!h.isHiveReadyForShot(.36),"Three out-of-window frames in a row revoke permission");

        // v47 hole: a steady tag at a DOWN or tipping height must never arm.
        for(double low:new double[]{30.5,35.6,39.3,44}) {
            h=new IsHiveReadyForShot(M,38);
            for(int i=1;i<=100;i++) {
                h.updateForAutonomous(new IsHiveReadyForShot.Frame(i,tag(38,low,0,60,M),tag(39,low+.2,0,60,M)),i*.02);
                check(!h.isHiveReadyForShot(i*.02),"A steady DOWN/tipping cell never qualifies: "+low);
            }
        }
        // Mostly DOWN with an occasional UP-looking frame never confirms.
        h=new IsHiveReadyForShot(M,38);
        for(int i=1;i<=60;i++) {
            h.updateForAutonomous(new IsHiveReadyForShot.Frame(i,tag(38,i%4==0?52:40,0,60,M)),i*.02);
            check(!h.isHiveReadyForShot(i*.02),"Three DOWN frames out of four never confirm UP");
        }
        h=new IsHiveReadyForShot(M,38);
        for(int i=1;i<=70;i++) {
            double now=i*.02;
            h.updateForAutonomous(new IsHiveReadyForShot.Frame(i,tag(41,60,10,60,M),tag(42,49.7,0,60,M)),now);
            check(!h.isHiveReadyForShot(now),"An in-band wrong-cell tag cannot arm");
        }

        PollenHiveRoute c=new PollenHiveRoute(true); c.v[0]=2;
        h=new IsHiveReadyForShot(M,38);
        PollenHiveAuto a=new PollenHiveAuto(c,h,PollenHiveAuto.Run.SHOT_ONLY); a.start(0);
        PollenHiveAuto.Output o=null;
        for(int i=1;i<=100;i++) {
            o=a.tick(i*.02,new IsHiveReadyForShot.Frame(i,tag(41,54.1,14,60,M)),0,0,0,1000,true);
            check(!o.feed&&o.intake==0,"Wait for the launcher without feeding");
        }
        check(o.decision.contains("hive_up_latched=true"),"High UP tag completes auto confirmation");
        o=a.tick(2.02,new IsHiveReadyForShot.Frame(101,tag(41,58,14,60,M)),0,0,0,1000,true);
        check(o.decision.contains("hive_up_latched=true"),"One odd frame during spin-up keeps the latch");
        o=a.tick(2.04,new IsHiveReadyForShot.Frame(102,tag(41,40,14,60,M)),0,0,0,1000,true);
        o=a.tick(2.06,new IsHiveReadyForShot.Frame(103,tag(41,40,14,60,M)),0,0,0,1000,true);
        o=a.tick(2.08,new IsHiveReadyForShot.Frame(104,tag(41,40,14,60,M)),0,0,0,1180,true);
        check(!o.feed,"A latch earned during spin-up cannot start feeding after permission is lost");
        for(int i=105;i<=200;i++) {
            o=a.tick(i*.02,new IsHiveReadyForShot.Frame(i,tag(41,54.1,14,60,M)),0,0,0,1180,true);
            if(o.feed) break;
        }
        check(o.feed&&o.intake==1,"A re-confirmed UP cell starts the volley at full intake");
    }

    private static void unsettledReturnParks() {
        PollenHiveRoute c=new PollenHiveRoute(false); c.v[0]=2;
        PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
        PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot();
        robot.leftLimit=PollenFieldGeometry.END_WALL_INCHES-9+PollenFieldGeometry.PRIMARY_START_X_INCHES; // side wall, v43 start
        PollenHiveAuto.Output o=null; double off=0; boolean secondFeed=false; Set<String> phases=new HashSet<>();
        for(int i=1;i<=1501;i++) {
            double t=i*.02;
            o=a.tick(t,frame(i,true),robot.x+off,robot.y,robot.h,o==null?0:o.launcher,true);
            if(o.phase.equals("SETTLE")) off=1; // odometry/robot ends 1 in off the shot pose
            phases.add(o.phase.split(":")[0]); secondFeed|=o.phase.equals("SECOND")&&o.feed;
            robot.step(o.forward,o.strafe,o.turn,false,.02);
            if(o.finished) break;
        }
        check(phases.contains("SETTLE")&&!secondFeed&&o.finished&&o.reason.equals("Finished departure"),"Unsettled return skips the second shot and still parks: "+o.reason+" "+phases);
    }
    private static void returnHeadingCorrection(double degrees) {
        PollenHiveAuto a=new PollenHiveAuto(new PollenHiveRoute(false),hive()); a.start(0);
        PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot();
        robot.leftLimit=PollenFieldGeometry.END_WALL_INCHES-9+PollenFieldGeometry.PRIMARY_START_X_INCHES; // side wall, v43 start
        boolean drifted=false,turned=false;
        for(int i=1;i<=1501;i++) {
            PollenHiveAuto.Output o=a.tick(i*.02,frame(i,true),robot.x,robot.y,robot.h,1180,true);
            if(o.phase.equals("SETTLE")&&!drifted) {
                // The v28 field run settled in position, then drifted just beyond two degrees.
                robot.h=Math.toRadians(degrees);
                robot.w=0;
                drifted=true;
            }
            if(o.phase.equals("ALIGN_SHOT")) {
                check(!a.checksHive()&&o.launcher==PollenConfig.LAUNCHER_TARGET_VELOCITY&&!o.feed&&o.intake==(o.turn==0?0:1)&&o.agitator==0
                        &&o.forward==0&&o.strafe==0,"Turn in place while pre-spinning and holding feed off");
                if(!turned&&o.turn!=0) {
                    check(Math.signum(o.turn)==Math.signum(degrees),"Turn toward zero from either side");
                    turned=true;
                }
            }
            if(o.phase.equals("SECOND")&&o.feed) {
                check(turned&&Math.abs(robot.h)<Math.toRadians(2),"Correct heading before second feed");
                return;
            }
            robot.step(o.forward,o.strafe,o.turn,false,.02);
            if(o.finished) throw new AssertionError("Never shot after return heading correction: "+o.reason);
        }
        throw new AssertionError("Never finished heading correction");
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
        check(capture(false,M,IsHiveReadyForShot.State.DOWN,35.6).problem().isEmpty(),"Physical DOWN verified by the calibration detector");
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
        check(Math.abs(PollenFieldGeometry.PRIMARY_START_X_INCHES-nodes[2].y+60.585)<1e-6,"Pickup center derived from side-wall clearance (v43 start)");
        Properties oldHeading=new Properties(); oldHeading.setProperty("transitHeading","-90");
        check(c.values.load(oldHeading).contains("transitHeading")&&c.values.get("transitHeading")==0,
                "Old saved transit heading cannot restore turns");
        c.validate();
        c.v[13]=-30; boolean rejected=false; try { c.validate(); } catch(IllegalArgumentException ex) { rejected=true; }
        check(rejected,"Route crossing end wall rejected");
        new PollenHiveRoute(true).validate();
    }
    private static void collectionTuning() {
        PollenHiveRoute primary=new PollenHiveRoute(false), secondary=new PollenHiveRoute(true);
        int sideways=secondary.values.index("collection.shakePower");
        secondary.values.edit(sideways,1);
        secondary.v[secondary.values.index("collection.down")]=.06;
        secondary.v[secondary.values.index("collection.intake")]=.85;
        PollenHiveRoute loaded=new PollenHiveRoute(true);
        check(loaded.values.load(secondary.values.overrides()).isEmpty(),"Saved flower settings reload");
        loaded.validate(true);
        PollenHiveSettings.Values flower=loaded.collectionSettings();
        check(flower.get("shakePower")==.80&&flower.get("down")==.06&&flower.get("intake")==.85&&flower.get("shakes")==2,
                "The collection sequence receives saved flower tuning and the beta cycle count");
        check(PollenCollectionSequence.settings(PollenCollectionSequence.Kind.FLOWER).get("shakePower")==.75,
                "Beta tuning leaves the isolated flower profile independent");
        primary.v[primary.values.index("collection.backoffIn")]=5;
        primary.v[primary.values.index("matchGardenPasses")]=2;
        check(primary.collectionSettings().get("backoffIn")==5&&primary.collectionSettings().get("repeats")==1,
                "Garden tuning and existing pass settings reach the same routine");
    }
    private static void secondaryGates() {
        PollenHiveRoute c=new PollenHiveRoute(true); c.v[0]=2;
        PollenHiveAuto a=new PollenHiveAuto(c,hive(),PollenHiveAuto.Run.SHOT_ONLY); a.start(0);
        for(int i=1;i<=100;i++) {
            boolean up=i<=50||i>=65;
            check(!a.tick(i*.02,up?frame(i,true):new IsHiveReadyForShot.Frame(i),0,0,0,1180,true).feed,
                    "Losing the cell resets the one-second UP timer");
        }
        PollenHiveAuto.Output o=null;
        for(int i=101;i<=140;i++) {
            o=a.tick(i*.02,frame(i,true),0,0,0,1000,true);
            check(!o.feed,"Must reach launcher speed to start");
        }
        check(o.decision.contains("hive_up_latched=true"),"Stable UP latches while spinning up");
        o=a.tick(2.82,frame(141,true),0,0,0,1180,true);
        check(o.feed,"Secondary starts once the launcher is ready");
        for(int i=142;i<=343;i++) {
            double now=i*.02;
            o=a.tick(now,i<180?frame(i,false):null,0,0,0,1050,true);
            check(o.feed==!o.finished,"Active volley continues through off-band readings, camera loss and low speed");
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
            o=a.tick(i*.02,i>1100?frame(i,true):new IsHiveReadyForShot.Frame(i),0,0,0,1180,true);
            check(!o.feed,"Skip a late volley that cannot finish");
            if(o.finished) break;
        }
        check(o.finished&&o.reason.contains("Insufficient time for full volley"),"Preserve the full-volley time reserve");
    }
    /** True when the launcher and agitator are off, or on only inside the 3 s clear-out window that
     * began on the tick `end`. Never feed, never in DRIVE. */
    private static boolean clearOutOnly(PollenHiveAuto.Output o,double t,double end,boolean driveOnly) {
        if(o.feed) return false;
        if(driveOnly||Double.isNaN(end)) return o.launcher==0&&o.agitator==0;
        return (o.launcher==0||t<end+3-1e-9)&&(o.agitator==0||t<end+3-1e-9);
    }
    /** After each match volley the agitator and launcher both run 3 s more and stop together, while
     * the robot drives on; intake and wheels are untouched; the park deadline cancels it; 0 disables it. */
    private static void clearOutAfterVolley() {
        for(boolean secondary:new boolean[]{false,true}) {
            PollenHiveRoute c=new PollenHiveRoute(secondary); c.v[0]=2;
            check(c.clearOutSeconds()==3,"Clear-out default: 3 s for agitator and launcher");
            PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
            PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot();
            if(!secondary) robot.leftLimit=48.84;
            PollenHiveAuto.Output o=null; String previous="";
            double[] ends={Double.NaN,Double.NaN}; int volleys=0; boolean movedWhileClearing=false;
            for(int i=1;i<=1501;i++) {
                double t=i*.02;
                o=a.tick(t,frame(i,true),robot.x,robot.y,robot.h,1180,true);
                String ph=o.phase.split(":")[0];
                if((previous.equals("FIRST")||previous.equals("SECOND"))&&!ph.equals(previous)) ends[volleys++]=t;
                double end=volleys==0?Double.NaN:ends[volleys-1];
                if(volleys>0&&!ph.equals("FIRST")&&!ph.equals("SECOND")&&!ph.equals("RETURN")&&!ph.equals("SETTLE")&&!ph.equals("ALIGN_SHOT")
                        &&!ph.equals("PARK")&&!ph.equals("DONE")) {
                    double since=t-end;
                    check(o.agitator==(since<3-1e-9?1:0),"Agitator clears for 3 s after the volley ("+ph+" +"+since+")");
                    check(o.launcher==(since<3-1e-9?PollenConfig.LAUNCHER_TARGET_VELOCITY:0),"Launcher clears for 3 s after the volley ("+ph+" +"+since+")");
                    check(o.agitator==0||o.launcher>0,"Agitator never runs into a stopped launcher");
                    check(!o.feed,"Clear-out is not a feed");
                    if(since<3-1e-9) check(o.intake==1,"Clear-out runs full intake");
                    movedWhileClearing|=since<3&&(o.forward!=0||o.strafe!=0);
                }
                if(ph.equals("PARK")) check(o.launcher==0&&o.agitator==0,"No clear-out while parking");
                previous=ph;
                robot.step(o.forward,o.strafe,o.turn,false,.02);
                if(secondary&&ph.equals("FLOWER")&&robot.x<5.1) { robot.x=5.1; robot.vf=0; }
                if(o.finished) break;
            }
            check(volleys==2&&movedWhileClearing&&o.reason.equals("Finished departure"),
                    "Both volleys clear out, the first while driving away, then park: "+volleys+" "+o.reason);
        }
        // The park deadline cancels a clear-out still running; 0 s turns it off.
        PollenHiveRoute c=new PollenHiveRoute(false); c.v[0]=2;
        c.v[c.values.index("clearOut")]=6;
        c.v[c.values.index("matchSecondFeed")]=5;
        PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
        PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot(); robot.leftLimit=48.84;
        boolean sawTailBeforePark=false,parked=false; String previous="";
        for(int i=1;i<=1501;i++) {
            PollenHiveAuto.Output o=a.tick(i*.02,frame(i,true),robot.x,robot.y,robot.h,1180,true);
            String ph=o.phase.split(":")[0];
            if(previous.equals("PARK_WAIT")&&ph.equals("PARK")) {
                check(o.launcher==0&&o.agitator==0,"Park deadline cancels the clear-out on its first tick");
                parked=true;
            }
            if(ph.equals("PARK_WAIT")) sawTailBeforePark|=o.agitator>0&&o.launcher>0;
            previous=ph;
            robot.step(o.forward,o.strafe,o.turn,false,.02);
            if(o.finished) break;
        }
        check(sawTailBeforePark&&parked,"A long clear-out still yields to the park deadline");
        c=new PollenHiveRoute(false); c.v[0]=2;
        c.v[c.values.index("clearOut")]=0;
        a=new PollenHiveAuto(c,hive()); a.start(0);
        previous="";
        for(int i=1;i<=600;i++) {
            PollenHiveAuto.Output o=a.tick(i*.02,frame(i,true),0,0,0,1180,true);
            if(previous.equals("FIRST")&&!o.phase.equals("FIRST")) {
                check(o.launcher==0&&o.agitator==0&&o.intake==0,"0 s restores the old all-off departure tick");
                break;
            }
            previous=o.phase;
        }
        c.v[c.values.index("clearOut")]=6.5; boolean rejected=false;
        try { new PollenHiveAuto(c,hive()); } catch(IllegalArgumentException ex) { rejected=true; }
        check(rejected,"Clear-out longer than 6 s is rejected at INIT");
    }
    /** Full simulated primary run; returns {second feed end s or NaN, finish s, volleys}. */
    private static double[] simulatePrimary(PollenHiveRoute c) {
        PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
        PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot(); robot.leftLimit=48.84;
        double feed2End=Double.NaN; int volleys=0; boolean wasFeed=false; PollenHiveAuto.Output o=null;
        for(int i=1;i<=1501;i++) {
            o=a.tick(i*.02,frame(i,true),robot.x,robot.y,robot.h,1180,true);
            if(wasFeed&&!o.feed) { volleys++; if(volleys==2) feed2End=i*.02; }
            wasFeed=o.feed;
            robot.step(o.forward,o.strafe,o.turn,false,.02);
            if(o.finished) return new double[]{feed2End,i*.02,volleys,o.reason.equals("Finished departure")?1:0};
        }
        return new double[]{feed2End,Double.NaN,volleys,0};
    }
    /** Beta Primary Shooter - Fast: own defaults, same route checks, both volleys, earlier finish. */
    private static void fastCycle() {
        PollenHiveRoute f=new PollenHiveRoute(false,false,true), m=new PollenHiveRoute(false);
        check(f.values.get("collectionEntryForward")==20.32&&f.values.get("collectionEntryLeft")==116
                &&f.values.get("matchFirstFeed")==2.5&&f.values.get("matchSecondFeed")==2.5&&f.values.changedCount()==0,
                "Fast mode defaults: entry 20.32/116 cm, feeds 2.5 s, shown as defaults (not CHANGED)");
        check(m.values.get("collectionEntryForward")==34&&m.values.get("collectionEntryLeft")==108
                &&m.values.get("matchFirstFeed")==3&&m.values.get("matchSecondFeed")==3,"Match primary defaults unchanged");
        boolean rejected=false;
        try { new PollenHiveRoute(true,false,true); } catch(IllegalArgumentException ex) { rejected=true; }
        check(rejected,"Fast mode is primary only");
        f.selectMatchAlliance(false); m.selectMatchAlliance(false); f.v[0]=2; m.v[0]=2;
        double[] fast=simulatePrimary(f), match=simulatePrimary(m);
        check(fast[2]==2&&fast[3]==1&&match[2]==2&&match[3]==1,"Both simulate two volleys and park: "+fast[2]+" "+match[2]);
        check(fast[0]<match[0]-1,"Fast mode's second volley ends over 1 s earlier in simulation: "+fast[0]+" vs "+match[0]);
        System.out.printf(Locale.US,"Fast mode simulated (assumed robot): second volley ends %.2f s vs %.2f s Match%n",fast[0],match[0]);
    }
    /** Fast mode: stopped 0.5-3 in off the spot after the return, it nudges back once and shoots;
     * Match skips the volley as before; a nudge that cannot finish gives up within 1.5 s and parks. */
    private static void returnNudge() {
        for(int variant=0;variant<3;variant++) {
            boolean fast=variant>0, stuck=variant==2;
            PollenHiveRoute c=new PollenHiveRoute(false,false,fast); c.selectMatchAlliance(false); c.v[0]=2;
            PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
            PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot(); robot.leftLimit=48.84;
            boolean pushed=false,nudgedSeen=false,second=false; String reason="";
            for(int i=1;i<=1501;i++) {
                double t=i*.02;
                PollenHiveAuto.Output o=a.tick(t,frame(i,true),robot.x,robot.y,robot.h,1180,true);
                // As the return arrives, shove the robot 1 in sideways off the spot (as if it coasted).
                if(!pushed&&o.phase.equals("SETTLE")) { robot.y+=1.0; robot.vl=0; pushed=true; }
                nudgedSeen|=o.reason.startsWith("Stopped")&&o.phase.startsWith("RETURN");
                second|=o.feed&&pushed;
                if(stuck&&nudgedSeen&&o.phase.startsWith("RETURN")) { robot.step(0,0,0,false,.02); }
                else robot.step(o.forward,o.strafe,o.turn,false,.02);
                reason=o.reason;
                if(o.finished) break;
            }
            if(!fast) check(!nudgedSeen&&!second,"Match does not nudge; it skips the volley as before");
            else if(!stuck) check(nudgedSeen&&second&&reason.equals("Finished departure"),"Fast mode nudges back once and fires the second volley: "+reason);
            else check(nudgedSeen&&!second&&reason.equals("Finished departure"),"A stuck nudge gives up and still parks: "+reason);
        }
    }
    /** Paced feed: the agitator holds while the launcher is below the set %, restarts at 96 %; 0 = old behavior. */
    private static void pacedFeed() {
        for(double pct:new double[]{0,92}) {
            PollenHiveRoute c=new PollenHiveRoute(false); c.selectMatchAlliance(false); c.v[0]=2;
            c.v[c.values.index("feedPausePct")]=pct;
            PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
            int held=0,fed=0; boolean resumedBeforeReady=false; double feedStart=Double.NaN,feedEnd=Double.NaN;
            for(int i=1;i<=400;i++) {
                double t=i*.02;
                // Launcher: full speed, then a ball dips it to 1000 at 1.0 s and it climbs 20 ticks/s per tick.
                double since=t-(Double.isNaN(feedStart)?99:feedStart);
                double v=since>=.5&&since<.5+.02*9?1000+20*((since-.5)/.02):1180;
                PollenHiveAuto.Output o=a.tick(t,frame(i,true),0,0,0,v,true);
                if(o.feed&&Double.isNaN(feedStart)) feedStart=t;
                if(!o.phase.equals("FIRST")) { feedEnd=t; break; }
                if(o.feed) { if(o.agitator==0) held++; else fed++; }
                if(o.feed&&o.agitator>0&&v<PollenConfig.LAUNCHER_TARGET_VELOCITY*.96&&since>.5) resumedBeforeReady|=held>0;
            }
            if(pct==0) check(held==0,"0 % feeds continuously through the dip");
            else check(held>0&&!resumedBeforeReady,"92 % holds the agitator through the dip and restarts at 96 %: held "+held);
            check(Math.abs(feedEnd-feedStart-3)<.1,"The volley clock keeps running while held: "+(feedEnd-feedStart));
        }
    }
    private static double primaryCycle(boolean driveOnly,boolean tipped,double travelPower) {
        PollenHiveRoute c=new PollenHiveRoute(false); c.v[0]=driveOnly?1:2;
        c.v[c.values.index("matchTravelPower")]=travelPower;
        IsHiveReadyForShot h=hive(); PollenHiveAuto a=new PollenHiveAuto(c,h); a.start(0);
        PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot();
        robot.leftLimit=PollenFieldGeometry.END_WALL_INCHES-9+PollenFieldGeometry.PRIMARY_START_X_INCHES; // side wall, v43 start
        Set<String> phases=new HashSet<>(); boolean collect=false,secondFeed=false,firstFeed=false,wallContact=false;
        PollenHiveAuto.Output o=null;
        double parkStarted=Double.NaN, finishedAt=Double.NaN, firstEnd=Double.NaN, secondEnd=Double.NaN;
        for(int i=1;i<=1501;i++) {
            double t=i*.02;
            double velocity=o==null?0:o.launcher;
            boolean up=!tipped;
            if(!phases.contains("RETURN")) check(!a.checksHive(),"Primary must skip Hive UP before returning");
            o=a.tick(t,up?frame(i,true):new IsHiveReadyForShot.Frame(i),robot.x,robot.y,robot.h,velocity,true);
            if(!driveOnly&&(o.forward!=0||o.strafe!=0||o.turn!=0||o.feed))
                check(o.intake==1,"Full intake whenever live auto drives, turns or feeds");
            phases.add(o.phase.split(":")[0]);
            if(o.phase.startsWith("OUTBOUND")&&Double.isNaN(firstEnd)) firstEnd=t;
            if(o.phase.equals("PARK_WAIT")&&Double.isNaN(secondEnd)) secondEnd=t;
            if(o.phase.startsWith("PARK:")&&Double.isNaN(parkStarted)) parkStarted=t;
            if(o.phase.startsWith("GARDEN")&&o.intake>0) collect=true;
            if(o.phase.startsWith("GARDEN")) {
                check(!o.phase.contains("Back off")&&!o.phase.contains("Inward pass 2"),"One forward garden pass");
                wallContact|=o.decision.contains("CONTACT LIMIT: node=Inward pass 1");
            }
            if(o.phase.startsWith("PARK:")) check(o.launcher==0&&o.agitator==0,"No shooting while parking");
            if(o.phase.equals("PARK_WAIT")) check(clearOutOnly(o,t,secondEnd,driveOnly),"Only the clear-out runs while waiting to park");
            if(o.phase.startsWith("OUTBOUND")||o.phase.startsWith("GARDEN")) check(clearOutOnly(o,t,firstEnd,driveOnly),
                    "Only the post-volley clear-out runs launcher/agitator on the trip, at "+t);
            if(o.phase.startsWith("RETURN")) check(o.launcher==(driveOnly?0:PollenConfig.LAUNCHER_TARGET_VELOCITY)&&o.agitator==0&&!o.feed,
                    "Pre-spin all the way back, with feed off");
            if(o.phase.equals("FIRST")&&o.feed) {
                check(!a.checksHive()&&h.lastTagCount==0,"Primary preload never classifies a camera frame");
                firstFeed=true;
            }
            if(o.phase.equals("SECOND")&&o.feed) secondFeed=true;
            if(o.feed) check(o.intake==1&&Math.hypot(robot.x,robot.y)<=.5&&Math.abs(robot.h)<=Math.toRadians(2),
                    "Feed at full intake only at restored shot pose");
            robot.step(o.forward,o.strafe,o.turn,false,.02);
            if(o.finished) {
                finishedAt=t;
                System.out.printf(Locale.US,"Hive simulated %s tipped=%s power=%.2f: %.2fs, park %.2fs, %s%n",
                        driveOnly?"DRIVE":"LIVE",tipped,travelPower,t,t-parkStarted,o.reason);
                break;
            }
        }
        check(wallContact,"Primary must contact the garden wall before its return; last="+o.reason);
        check(phases.contains("SECOND"),"Primary must return for second shot; last="+o.reason+" pose="+robot.x+","+robot.y+","+robot.h);
        check(phases.contains("PARK")&&o.finished&&o.reason.equals("Finished departure"),
                "Default cycle attempts and completes park: "+o.reason);
        if(driveOnly) check(!firstFeed&&!secondFeed&&!collect,"Drive-only disables all mechanism actions");
        else { check(firstFeed&&collect,"Primary fires and collects"); check(secondFeed!=tipped,"Second shot respects current hive state"); }
        return finishedAt;
    }
    /** v46: "Beta flower forward-back cycles" = 0 turns the flower trip off. The secondary waits
     * for UP, shoots, then stays at its start (drainer up, wheels still) until the park deadline,
     * then parks. With the default 1 the flower trip still runs. */
    private static void secondaryFlowerOff() {
        PollenHiveRoute on=new PollenHiveRoute(true);
        check(on.values.get("matchFlowerCleanup")==1&&!on.flowerDrainOff(),"Default: flower drain on");
        PollenHiveRoute primary=new PollenHiveRoute(false);
        primary.v[primary.values.index("matchFlowerCleanup")]=0;
        check(!primary.flowerDrainOff(),"The switch is secondary-only");
        PollenHiveRoute c=new PollenHiveRoute(true);
        c.v[c.values.index("matchFlowerCleanup")]=0;
        check(c.flowerDrainOff(),"Forward-back cycles 0 = flower drain off");
        c.validate(true);
        double deadline=30-c.collectionParkSeconds()-c.parkMargin();
        PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
        PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot();
        Set<String> phases=new HashSet<>();
        boolean fed=false; double parkAt=Double.NaN, maxMoveBeforePark=0;
        PollenHiveAuto.Output o=null;
        for(int i=1;i<=1501;i++) {
            double t=i*.02;
            o=a.tick(t,t>=4?frame(i,true):new IsHiveReadyForShot.Frame(i),robot.x,robot.y,robot.h,1180,true);
            String phase=o.phase.split(":")[0];
            phases.add(phase);
            if(phase.equals("FIRST")) fed|=o.feed;
            if(phase.equals("PARK")&&Double.isNaN(parkAt)) parkAt=t;
            if(Double.isNaN(parkAt)) {
                maxMoveBeforePark=Math.max(maxMoveBeforePark,Math.hypot(robot.x,robot.y));
                check(!o.drainerDown&&Math.abs(o.forward)+Math.abs(o.strafe)+Math.abs(o.turn)<1e-9,
                        "Flower off: no drive and no drainer before park ("+o.phase+" at "+t+")");
            }
            robot.step(o.forward,o.strafe,o.turn,false,.02);
            if(o.finished) break;
        }
        check(fed&&!phases.contains("OUTBOUND")&&!phases.contains("FLOWER")&&!phases.contains("RETURN")&&!phases.contains("SECOND"),
                "Flower off: one volley, no flower trip, no return volley: "+phases);
        check(maxMoveBeforePark<.01&&Math.abs(parkAt-deadline)<.05&&o.finished&&o.reason.equals("Finished departure"),
                "Flower off: stays at the start until the park deadline ("+deadline+" s), then parks: "+parkAt+" "+o.reason);
    }
    private static void secondaryFlower(boolean upAfterReturn) {
        PollenHiveRoute c=new PollenHiveRoute(true); c.v[21]=0; c.v[3]=5; // Hub's saved calibration duration
        c.v[c.values.index("collection.intake")]=.85;
        c.v[c.values.index("collection.shakePower")]=.15;
        check(c.feedDuration(false,true)==3&&c.feedDuration(true,true)==3&&c.feedDuration(false,false)==5,
                "The beta uses three-second feeds and preserves the calibration override");
        c.v[c.values.index("matchFirstFeed")]=5; // Reproduce the long first volley from the field trace
        PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
        PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot();
        Set<String> phases=new HashSet<>();
        Set<String> cleanup=new HashSet<>(), swings=new HashSet<>();
        boolean firstFeed=false,secondFeed=false,seated=false;
        PollenHiveAuto.Output o=null; double firstEnd=Double.NaN;
        for(int i=1;i<=1501;i++) {
            boolean up=phases.contains("RETURN")?upAfterReturn:i*.02>=6.1;
            o=a.tick(i*.02,up?frame(i,true):new IsHiveReadyForShot.Frame(i),robot.x,robot.y,robot.h,1180,true);
            String phase=o.phase.split(":")[0];
            phases.add(phase);
            if(phase.equals("OUTBOUND")&&Double.isNaN(firstEnd)) firstEnd=i*.02;
            if(o.phase.startsWith("FLOWER: ")&&!o.phase.equals("FLOWER: ")) {
                check(!a.checksHive()&&clearOutOnly(o,i*.02,firstEnd,false),"No aiming at the flower; only the clear-out shoots");
                check(o.intake==1&&o.drainerDown,"Full intake overrides saved power during the flower process");
                if(o.phase.contains("Ingest forward")) cleanup.add(o.phase);
                if(o.phase.startsWith("FLOWER: Left ")||o.phase.startsWith("FLOWER: Right ")) {
                    check(Math.abs(o.strafe)<=.150001,"Tuned sideways power limits swing motion");
                    swings.add(o.phase);
                }
                if(o.decision.contains("CONTACT LIMIT")&&o.phase.contains("Seat drainer")) seated=true;
            } else check(!o.drainerDown,"Drainer raised outside flower process");
            if(o.phase.equals("FIRST")) firstFeed|=o.feed;
            if(o.phase.equals("SECOND")) {
                check(phases.contains("RETURN")&&Math.hypot(robot.x,robot.y)<=.5,"Return volley only at the shooting position");
                check(!a.checksHive(),"v39: the secondary's return volley does not wait for the camera");
                secondFeed|=o.feed;
            }
            robot.step(o.forward,o.strafe,o.turn,false,.02);
            if(phase.equals("FLOWER")&&robot.x<5.1) { robot.x=5.1; robot.vf=0; }
            if(o.finished) break;
        }
        // v39: the cell needs about 8 balls to tip, so it is still UP after our 4 preloads. The return
        // volley fires whatever the camera sees after the return (upAfterReturn false = hidden).
        check(o.finished&&firstFeed&&seated&&cleanup.size()==1&&swings.size()==4&&phases.contains("SECOND")&&secondFeed,
                "Secondary drains, returns and fires without a camera check (camera UP="+upAfterReturn+"): "
                +phases+" "+cleanup+" "+swings+" "+o.reason);
    }
    private static void lateCollectionAndPark() {
        PollenHiveRoute c=new PollenHiveRoute(false); c.v[0]=2;
        check(c.collectionOutbound()[1].power==1&&c.collectionReturn(0,48)[1].power==1
                &&c.collectionPark()[2].power==1&&c.park()[2].power==.6,"Full power applies to both beta transit and park");
        PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
        PollenHiveAuto.Output o=null;
        boolean fed=false;
        for(int i=1;i<=950;i++) {
            double t=i*.02;
            double velocity=t<17?0:1180;
            o=a.tick(t,frame(i,true),0,0,0,velocity,true);
            fed|=o.feed;
            if(o.phase.startsWith("OUTBOUND")) break;
        }
        // The first volley completes past the former preload window. The trip estimate still
        // exceeds the available time, but the first commanded drive leg should be outbound.
        o=a.tick(19.02,frame(951,true),0,0,0,1180,true);
        check(fed&&o.phase.equals("FIRST")&&o.feed,"Late first volley continues feeding");
        for(int i=952;i<=1010;i++) {
            o=a.tick(i*.02,frame(i,true),0,0,0,1180,true);
            if(o.phase.startsWith("OUTBOUND")) break;
        }
        check(o.phase.startsWith("OUTBOUND"),"Late first volley still starts collection");

        c.v[c.values.index("matchSecondFeed")]=12;
        c.values.edit(c.values.index("parkMargin"),1);
        check(c.parkMargin()==1.5,"Park margin defaults to one second and advances by half a second");
        a=new PollenHiveAuto(c,hive()); a.start(0);
        PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot();
        robot.leftLimit=PollenFieldGeometry.END_WALL_INCHES-9+PollenFieldGeometry.PRIMARY_START_X_INCHES; // side wall, v43 start
        boolean secondFeed=false, startedPark=false;
        for(int i=1;i<=1501;i++) {
            double t=i*.02;
            o=a.tick(t,frame(i,true),robot.x,robot.y,robot.h,1180,true);
            secondFeed|=o.phase.equals("SECOND")&&o.feed;
            if(!startedPark&&o.phase.startsWith("PARK:")) {
                check(secondFeed&&o.reason.contains("deadline during SECOND")&&Math.abs(t-23.04)<.03,
                        "Measured primary park time plus the tuned margin interrupts second feed: "+t+" "+o.reason);
                check(o.launcher==0&&o.intake==1&&o.agitator==0&&!o.feed&&(o.forward!=0||o.strafe!=0),
                        "First park tick stops shooting and starts driving");
                startedPark=true;
            }
            if(o.finished) break;
            robot.step(o.forward,o.strafe,o.turn,false,.02);
        }
        check(startedPark&&o.reason.equals("Finished departure")&&o.forward==0&&o.strafe==0,
                "Interrupted volley parks before the cutoff: "+o.reason);

        a=new PollenHiveAuto(c,hive()); a.start(0);
        o=a.tick(.02,frame(1,true),0,0,0,1180,false);
        check(o.finished&&o.launcher==0&&o.intake==0,"Pinpoint not READY latches a stop");
        double[] actual=PollenFacing.wheelPowers(.3,.1,.2);
        double[] expected=PollenSequence.wheelPowers(-.3,-.1,.2);
        for(int i=0;i<4;i++) check(actual[i]==expected[i],"Rear-facing conversion matches legacy boundary");
    }
    private static void slowMatchPark(boolean secondary, double power) {
        PollenHiveRoute route = new PollenHiveRoute(secondary);
        route.v[route.values.index("matchTravelPower")] = power;
        PollenHiveAuto auto = new PollenHiveAuto(route, hive());
        PollenOdometryTest.SimRobot robot = new PollenOdometryTest.SimRobot();
        robot.leftLimit = secondary ? Double.POSITIVE_INFINITY
                : PollenFieldGeometry.END_WALL_INCHES - 9 + route.startXInches();
        auto.start(0);
        PollenHiveAuto.Output output = null;
        for (int i = 1; i <= 1500; i++) {
            output = auto.tick(i * .02, null, robot.x, robot.y, robot.h, 1180, true);
            if (output.finished) break;
            robot.step(output.forward, output.strafe, output.turn, false, .02);
        }
        PollenHiveRoute.Node[] park = route.collectionPark();
        PollenHiveRoute.Node slot = park[park.length - 1];
        check(output.reason.equals("Finished departure") && Math.hypot(robot.x-slot.x, robot.y-slot.y)<=.5,
                "Slow park must reach the slot before cutoff: secondary=" + secondary + " power=" + power
                + " end=" + robot.x + "/" + robot.y + " " + output.reason);
    }
    private static void motionFaults() {
        PollenHiveMotion m=new PollenHiveMotion(new PollenHiveRoute(false).garden());
        PollenHiveMotion.Output o=null;
        for(int i=1;i<=50;i++) o=m.tick(i*.02,0,0,0);
        check(o.fault!=null,"Stuck motion stops");
        PollenHiveRoute c=new PollenHiveRoute(true); c.v[21]=0;
        PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
        for(int i=1;i<=1501;i++) oDummy=a.tick(i*.02,null,0,0,0,0,true);
        check(oDummy.finished&&oDummy.reason.equals("30 second cutoff during FIRST"),"Deadline stops even with no observation");
    }
    private static void parkFromCollectionLane() {
        PollenHiveRoute c=new PollenHiveRoute(false);
        c.v[c.values.index("matchFirstFeed")]=17;
        PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
        PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot();
        robot.leftLimit=PollenFieldGeometry.END_WALL_INCHES-9+PollenFieldGeometry.PRIMARY_START_X_INCHES; // side wall, v43 start
        for(int i=1;i<=1501;i++) {
            double t=i*.02;
            PollenHiveAuto.Output o=a.tick(t,frame(i,true),robot.x,robot.y,robot.h,1180,true);
            if(o.phase.startsWith("PARK:")) {
                check(t<23.54&&o.phase.contains("CLEAR COLLECTION SITE")&&Math.hypot(robot.x,robot.y)>1.5,
                        "An away pose budgets the clearance route before parking: "+o.phase+" at "+t);
                check(o.intake==1&&o.launcher==0&&!o.drainerDown,"Departure raises the drainer and keeps full intake while driving");
                return;
            }
            robot.step(o.forward,o.strafe,o.turn,false,.02);
        }
        throw new AssertionError("Never attempted park from the collection lane");
    }
    /** 2026-11-19 secondary trace: the park deadline came on the last return leg, 7 in from the
     * shot, and the park rebuilt the whole return and drove 7 in back out first. It must finish
     * the legs left instead. */
    private static void parkFinishesReturnLegs() {
        int lastLeg=0;
        for(double feed=3;feed<=9;feed+=.25) {
            PollenHiveRoute c=new PollenHiveRoute(true);
            c.v[c.values.index("matchFirstFeed")]=feed;
            PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
            PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot();
            String before="";
            double parkX=Double.NaN, maxX=Double.NEGATIVE_INFINITY;
            boolean clear=false;
            for(int i=1;i<=1501;i++) {
                PollenHiveAuto.Output o=a.tick(i*.02,i*.02>=6.1?frame(i,true):new IsHiveReadyForShot.Frame(i),robot.x,robot.y,robot.h,1180,true);
                if(o.phase.startsWith("PARK:")) {
                    if(Double.isNaN(parkX)) {
                        if(!before.equals("RETURN: RETURN SHOOT POSITION")) break;
                        parkX=robot.x; lastLeg++;
                        check(o.reason.contains("during RETURN"),"Deadline arrives on the return: "+o.reason);
                    }
                    clear|=o.phase.equals("PARK: PARK CLEAR");
                    if(!clear) {
                        maxX=Math.max(maxX,robot.x);
                        check(!o.phase.contains("CLEAR COLLECTION SITE")&&!o.phase.contains("RETURN ACROSS"),
                                "Park never replays finished return legs: "+o.phase+" feed "+feed);
                    }
                }
                if(o.finished) break;
                before=o.phase;
                robot.step(o.forward,o.strafe,o.turn,false,.02);
                if(o.phase.startsWith("FLOWER")&&robot.x<5.1) { robot.x=5.1; robot.vf=0; }
            }
            if(!Double.isNaN(parkX)) check(maxX<=parkX+.5,
                    "Park from the last return leg never backs out toward the flower: "+parkX+" -> "+maxX);
        }
        check(lastLeg>0,"Some first-feed length puts the park deadline on the last return leg");
    }
    /** 2026-11-19 primary: in position 0.44 in from the garden point but 2.3 deg crooked, the
     * proportional 0.03 turn never moved the robot and the auto sat until STOP. A robot that only
     * turns above 0.17 power must still finish; one that cannot turn at all must not hang. */
    private static void squaresUpInPlace() {
        PollenHiveRoute.Node[] leg={new PollenHiveRoute.Node("POINT",10,0,0,.6,false,false,0)};
        for(double threshold:new double[]{.12,.17}) {
            PollenHiveMotion m=new PollenHiveMotion(leg);
            double hd=Math.toRadians(-2.3), first=Double.NaN; PollenHiveMotion.Output o=null; int i;
            for(i=1;i<=200;i++) {
                o=m.tick(i*.02,10-.3,.3,hd);
                check(o.fault==null,"Squaring never faults: "+o.fault);
                if(o.done) break;
                if(Double.isNaN(first)&&o.turn!=0) first=Math.abs(o.turn);
                if(Math.abs(o.turn)>=threshold) hd-=o.turn*3*.02; // clockwise for +turn
            }
            check(first>=PollenOdometryDrive.MIN_TURN_POWER,"In-place squaring starts with a real turn: "+first);
            check(o.done&&Math.abs(Math.toDegrees(hd))<=2&&i*.02<2.5,
                    "A crooked in-position stop squares up and moves on (stiction "+threshold+"): "+Math.toDegrees(hd)+" deg at "+i*.02+" s");
        }
        PollenHiveMotion m=new PollenHiveMotion(leg);
        PollenHiveMotion.Output o=null; int i;
        for(i=1;i<=400;i++) { o=m.tick(i*.02,10-.3,.3,Math.toRadians(-2.3)); if(o.done||o.fault!=null) break; }
        check(o.done&&o.fault==null&&i*.02<=3.2,"A robot that cannot turn keeps going after "+PollenHiveMotion.SQUARE_GIVE_UP_S+" s, never hangs");
    }
    /** Beta Primary Shooter - Angled Left: drive straight to 8 in away / 8 in left, turn to the
     * computed aim, shoot; garden; back to the same spot and aim; shoot; park. */
    private static void angledPrimary() {
        check(Math.abs(PollenFieldGeometry.aimDegrees(0,0))<1e-9,"v43: from the start the launcher already points at the cell's middle");
        boolean rejected=false;
        try { new PollenHiveRoute(true,true,false,true); } catch(IllegalArgumentException ex) { rejected=true; }
        check(rejected,"Angled is not a make-room mode");
        check(Math.abs(PollenFieldGeometry.cellLeftOfStartInches(false)-2.7416)<.001
                &&Math.abs(PollenFieldGeometry.cellLeftOfStartInches(true)+2.713)<.001,
                "v43: cell middle 2.74 in left of the primary start (1 in from the seam), 2.71 in right of the secondary start");
        PollenHiveRoute c=new PollenHiveRoute(false,false,false,true);
        check(c.values.get("shotLeftIn")==10.5&&c.values.get("shotAwayIn")==8,"Angled spot defaults to 10.5 in left of the v43 start, 8 in away");
        int aimRow=c.values.index("aimTargetLeftIn"); c.values.edit(aimRow,-1);
        check(c.values.get("aimTargetLeftIn")==-2.5,"Aim target moves in 0.5 in steps and can go further negative");
        c.values.reset(aimRow);
        check(c.collectionSettings().get("intake")==1,"Angled rows do not disturb the garden settings");
        c.validate(true);
        PollenHiveRoute.Node[] to=c.toShot();
        check(to.length==1&&Math.abs(to[0].x-8)<1e-9&&Math.abs(to[0].y-10.5)<1e-9&&to[0].h==0,"One straight leg to the spot, no turn on the way");
        check(c.collectionPark().length==2&&c.collectionReturn(3,48)[1].name.equals("RETURN SHOOT POSITION"),
                "Angled park skips PARK CLEAR; the return ends at the spot");
        PollenHiveRoute bad=new PollenHiveRoute(false,false,false,true);
        bad.v[bad.values.index("shotAwayIn")]=3; bad.v[bad.values.index("shotLeftIn")]=24;
        rejected=false; try { bad.validate(true); } catch(IllegalArgumentException ex) { rejected=true; }
        check(rejected,"A spot too close to the wall for the aim turn fails INIT");
        double aim=Math.toRadians(PollenFieldGeometry.aimDegrees(8,10.5,PollenConfig.AIM_TARGET_LEFT_IN,
                PollenFieldGeometry.cellLeftOfStartInches(false)));
        {   // v43: the LAUNCHER's line (2.74 in left of center) passes 2 in right of the cell's middle.
            double L=PollenFieldGeometry.LAUNCHER_LEFT_OF_CENTER_INCHES;
            double lf=8-L*Math.sin(aim), ll=10.5+L*Math.cos(aim);
            double cf=PollenFieldGeometry.hiveOpeningForwardInches()-lf, cl=PollenFieldGeometry.cellLeftOfStartInches(false)-2-ll;
            check(c.values.get("aimTargetLeftIn")==-2&&Math.abs(cf*Math.sin(aim)-cl*Math.cos(aim))<1e-9&&Math.abs(c.shotHeading()-aim)<1e-12
                    &&Math.toDegrees(aim)<-14&&Math.toDegrees(aim)>-20,
                    "Angled primary aims the launcher 2 in right of the cell's middle: "+Math.toDegrees(aim));
        }
        IsHiveReadyForShot h=hive(); PollenHiveAuto a=new PollenHiveAuto(c,h); a.start(0);
        PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot();
        robot.leftLimit=PollenFieldGeometry.END_WALL_INCHES-9+PollenFieldGeometry.PRIMARY_START_X_INCHES; // side wall, v43 start
        java.util.List<String> order=new java.util.ArrayList<>();
        boolean firstFeed=false,secondFeed=false,spunUp=false,collect=false;
        PollenHiveAuto.Output o=null;
        for(int i=1;i<=1501;i++) {
            double t=i*.02;
            o=a.tick(t,frame(i,true),robot.x,robot.y,robot.h,o==null?0:o.launcher,true);
            String phase=o.phase.split(":")[0];
            if(order.isEmpty()||!order.get(order.size()-1).equals(phase)) order.add(phase);
            if(phase.equals("TO_SHOT")&&o.launcher==PollenConfig.LAUNCHER_TARGET_VELOCITY) spunUp=true;
            if(o.phase.startsWith("GARDEN")&&o.intake>0) collect=true;
            if(o.feed) {
                check(Math.hypot(robot.x-8,robot.y-10.5)<=(o.phase.equals("FIRST")?.5:PollenHiveAuto.RETURN_SHOT_TOLERANCE_IN)
                        &&Math.abs(PollenOdometryDrive.wrap(robot.h-aim))<=Math.toRadians(2),
                        "Angled volleys only at the spot and aimed: "+robot.x+","+robot.y+","+Math.toDegrees(robot.h)+" at "+t);
                if(o.phase.equals("FIRST")) firstFeed=true;
                if(o.phase.equals("SECOND")) secondFeed=true;
            }
            robot.step(o.forward,o.strafe,o.turn,false,.02);
            if(o.finished) break;
        }
        check(spunUp,"Flywheel spins up while driving to the spot: "+order+" "+o.reason);
        check(order.indexOf("TO_SHOT")==0&&order.indexOf("ALIGN_SHOT")>order.indexOf("TO_SHOT")
                &&order.indexOf("FIRST")>order.indexOf("ALIGN_SHOT"),"Drive, turn, then shoot: "+order);
        check(firstFeed&&collect&&secondFeed&&o.reason.equals("Finished departure"),
                "Angled primary shoots, drains the garden, shoots again from the spot and parks: "+order+" "+o.reason);
    }
    /** v52: the primary angled return stopped 0.58-0.65 in off the spot on the field and skipped a
     * loaded volley. Within 1 in the return volley fires; further off it is still skipped. */
    private static void angledPrimaryReturnOff(double offIn,boolean expectShot) {
        PollenHiveRoute c=new PollenHiveRoute(false,false,false,true);
        PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
        PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot();
        robot.leftLimit=PollenFieldGeometry.END_WALL_INCHES-9+PollenFieldGeometry.PRIMARY_START_X_INCHES;
        boolean returned=false,shifted=false,secondFeed=false; String skipped="";
        PollenHiveAuto.Output o=null;
        for(int i=1;i<=1501;i++) {
            o=a.tick(i*.02,frame(i,true),robot.x,robot.y,robot.h,o==null?0:o.launcher,true);
            if(o.phase.startsWith("RETURN")) returned=true;
            if(returned&&!shifted&&o.phase.equals("SETTLE")) {
                robot.x-=offIn; robot.vf=0; shifted=true; // stopped short of the spot, as on the field
            }
            if(o.phase.equals("SECOND")&&o.feed) secondFeed=true;
            if(o.reason.startsWith("Return pose did not settle")) skipped=o.reason;
            robot.step(o.forward,o.strafe,o.turn,false,.02);
            if(o.finished) break;
        }
        check(shifted&&secondFeed==expectShot&&(expectShot||!skipped.isEmpty())&&o.reason.equals("Finished departure"),
                "Return "+offIn+" in off the spot: second volley="+secondFeed+" (expected "+expectShot+") "+skipped+" "+o.reason);
    }
    /** v52: the alliance an auto saves for TeleOp, and Circle aiming only at that alliance. */
    private static void allianceForTeleOp() {
        java.util.Properties p=PollenAlliance.toProperties(PollenAlliance.BLUE,1000,"PollenHivePrimaryAngledAuto");
        PollenAlliance.Saved s=PollenAlliance.parse(p);
        check(s!=null&&s.side==PollenAlliance.BLUE&&s.wallMs==1000&&s.source.equals("PollenHivePrimaryAngledAuto"),"Alliance round trip");
        check(s.describe(1000+12*60000).equals("BLUE, from PollenHivePrimaryAngledAuto 12 min ago"),"Alliance age: "+s.describe(1000+12*60000));
        check(PollenAlliance.parse(new java.util.Properties())==null,"No file content = no alliance");
        java.util.Properties bad=new java.util.Properties(); bad.setProperty("alliance","GREEN");
        check(PollenAlliance.parse(bad)==null,"Unknown alliance is ignored");
        boolean rejected=false; try { PollenAlliance.toProperties(0,1,"x"); } catch(IllegalArgumentException ex) { rejected=true; }
        check(rejected,"AUTO (detect) is never saved");
        check(java.util.Arrays.equals(PollenAlliance.cells(1),new int[]{30,34})&&java.util.Arrays.equals(PollenAlliance.cells(2),new int[]{38,42})
                &&PollenAlliance.cells(0).length==4,"Alliance cells");
        // A red cell straight ahead, a blue cell 30 in to the side.
        IsHiveReadyForShot.Frame f=new IsHiveReadyForShot.Frame(1,tag(34,49.7,0,60,M),tag(35,49.7,4,60,M),
                tag(42,49.7,30,60,M),tag(43,49.7,34,60,M));
        check(PollenHiveAim.bearing(f,M,0).cluster==34,"Any alliance: nearest cell");
        check(PollenHiveAim.bearing(f,M,0,PollenAlliance.cells(PollenAlliance.BLUE)).cluster==42,"Blue: aims at the blue cell even if red is nearer");
        check(PollenHiveAim.bearing(new IsHiveReadyForShot.Frame(2,tag(34,49.7,0,60,M)),M,0,PollenAlliance.cells(PollenAlliance.BLUE))==null,
                "Blue with only red tags in view: no aim");
        PollenHiveAim aim=new PollenHiveAim(); aim.setCells(PollenAlliance.cells(PollenAlliance.BLUE));
        String e=aim.start(0,PollenHiveAim.Source.CAMERA,0,Double.NaN,Double.NaN,false,new IsHiveReadyForShot.Frame(3,tag(34,49.7,0,60,M)),0,M);
        check(e.startsWith("AIM_REFUSED")&&aim.lastResult().contains("alliance"),"Circle refuses when only the other alliance is in view: "+aim.lastResult());
    }
    /** v41: Beta Secondary Shooter - Angled. Drive to the spot, turn, wait for UP, shoot, flower,
     * back to the spot, shoot again with no camera wait, park straight from the spot. */
    private static void angledSecondary(boolean upAfterReturn) {
        PollenHiveRoute c=new PollenHiveRoute(true,false,false,true);
        check(c.values.get("shotLeftIn")==-10.5&&c.values.get("shotAwayIn")==8,"Secondary angled spot: 10.5 in right, 8 in away");
        c.validate(true);
        double aim=Math.toRadians(PollenFieldGeometry.aimDegrees(8,-10.5,-2,PollenFieldGeometry.cellLeftOfStartInches(true)));
        check(Math.abs(c.shotHeading()-aim)<1e-12&&Math.toDegrees(aim)>4.3&&Math.toDegrees(aim)<4.9,
                "v43: secondary angled turns the launcher about 4.6 deg left toward the cell: "+Math.toDegrees(aim));
        check(c.collectionOutbound()[0].name.equals("ALIGN WITH FLOWER")&&c.collectionPark().length==1
                &&c.collectionPark()[0].name.equals("PARK"),"One leg to the flower; park straight from the spot");
        PollenHiveRoute bad=new PollenHiveRoute(true,false,false,true);
        bad.v[bad.values.index("shotLeftIn")]=2; // toward the midline: the robot would cross it
        boolean rejected=false; try { bad.validate(true); } catch(IllegalArgumentException ex) { rejected=true; }
        check(rejected,"A secondary spot across the alliance midline fails INIT");
        int[] order=c.menuOrder(PollenHiveAuto.Run.MATCH);
        java.util.List<String> keys=new java.util.ArrayList<>();
        for(int i=0;i<14;i++) keys.add(c.values.rows[order[i]].key);
        check(keys.contains("shotLeftIn")&&keys.contains("shotAwayIn")&&keys.contains("aimTargetLeftIn")&&keys.get(1).equals("launcher"),
                "Secondary angled: spot and aim rows with the rows to check: "+keys);
        PollenHiveAuto a=new PollenHiveAuto(c,new IsHiveReadyForShot(M,38)); a.start(0);
        PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot();
        java.util.List<String> phases=new java.util.ArrayList<>();
        boolean firstFeed=false,secondFeed=false,spunUp=false,drained=false,returning=false;
        double shotX=8,shotY=-10.5;
        PollenHiveAuto.Output o=null;
        for(int i=1;i<=1501;i++) {
            double t=i*.02;
            boolean up=phases.contains("RETURN")?upAfterReturn:t>=4;
            IsHiveReadyForShot.Frame frame=up?new IsHiveReadyForShot.Frame(i,tag(41,54.1,10,60,M))
                    :new IsHiveReadyForShot.Frame(i);
            o=a.tick(t,frame,robot.x,robot.y,robot.h,o==null?0:o.launcher,true);
            String phase=o.phase.split(":")[0];
            if(phases.isEmpty()||!phases.get(phases.size()-1).equals(phase)) phases.add(phase);
            boolean moving=o.forward!=0||o.strafe!=0||o.turn!=0;
            if(phase.equals("TO_SHOT")||phase.equals("RETURN")||phase.equals("ALIGN_SHOT"))
                check(o.launcher==PollenConfig.LAUNCHER_TARGET_VELOCITY&&!o.drainerDown&&(!moving||o.intake==1),
                        "Angled secondary pre-spins with scoop up and full intake on each moving approach");
            if(phase.equals("TO_SHOT")&&moving) spunUp=true;
            if(phase.equals("RETURN")&&moving) returning=true;
            if(o.phase.startsWith("FLOWER")&&o.drainerDown) drained=true;
            if(phase.equals("FIRST")&&t<4) check(!o.feed,"First volley waits for our cell UP");
            if(o.feed&&!o.phase.startsWith("OUTBOUND")&&!o.phase.startsWith("FLOWER")&&!phase.equals("PARK_WAIT")) {
                check(Math.hypot(robot.x-shotX,robot.y-shotY)<=.5&&Math.abs(PollenOdometryDrive.wrap(robot.h-aim))<=Math.toRadians(2),
                        "Secondary angled volleys only at the spot and aimed: "+robot.x+","+robot.y+","+Math.toDegrees(robot.h)+" at "+t);
                if(phase.equals("FIRST")) firstFeed=true;
                if(phase.equals("SECOND")) secondFeed=true;
            }
            robot.step(o.forward,o.strafe,o.turn,false,.02);
            if(phase.equals("FLOWER")&&robot.x<5.1) { robot.x=5.1; robot.vf=0; }
            if(o.finished) break;
        }
        check(spunUp&&phases.indexOf("TO_SHOT")==0&&phases.indexOf("FIRST")>phases.indexOf("ALIGN_SHOT"),"Drive, turn, then wait and shoot: "+phases);
        check(firstFeed&&drained&&returning&&secondFeed&&o.reason.equals("Finished departure"),
                "Secondary angled shoots, drains, shoots again (camera UP="+upAfterReturn+") and parks: "+phases+" "+o.reason);
    }
    /** A failed first aim must not send a still-loaded secondary to the flower. */
    private static void angledSecondaryMissesPreload(boolean failTurn) {
        PollenHiveRoute c = new PollenHiveRoute(true, false, false, true);
        c.beginMatchSetup();
        c.selectMatchAlliance(false);
        IsHiveReadyForShot detector = new IsHiveReadyForShot(M, IsHiveReadyForShot.allowedCells(true, c.side()));
        PollenHiveAuto auto = new PollenHiveAuto(c, detector);
        auto.start(0);
        PollenOdometryTest.SimRobot robot = new PollenOdometryTest.SimRobot();
        boolean injected = false, collected = false, fed = false, parked = false;
        PollenHiveAuto.Output o = null;
        for (int i = 1; i <= 1500; i++) {
            // The red secondary cell's tags stay hidden while we simulate the failed aim.
            IsHiveReadyForShot.Frame hidden = new IsHiveReadyForShot.Frame(i);
            o = auto.tick(i * .02, hidden, robot.x, robot.y, robot.h, 1180, true);
            collected |= o.phase.startsWith("OUTBOUND") || o.drainerDown;
            fed |= o.feed || o.agitator != 0;
            parked |= o.phase.startsWith("PARK:");
            if (o.finished) break;
            robot.cannotTurnInPlace = failTurn && o.phase.equals("ALIGN_SHOT") && !collected;
            robot.step(o.forward, o.strafe, o.turn, false, .02);
            if (robot.cannotTurnInPlace) injected = true;
            if (!failTurn && !injected && o.phase.equals("SETTLE")) {
                robot.x += .8;  // just outside shot tolerance, no longer settling at the spot
                injected = true;
            }
            if (o.phase.startsWith("FLOWER") && robot.x < 5.1) { robot.x = 5.1; robot.vf = 0; }
        }
        check(injected && !collected && !fed && parked && o.reason.equals("Finished departure"),
                "Skipped secondary preload must park with four balls: failTurn=" + failTurn
                + " collected=" + collected + " fed=" + fed + " parked=" + parked + " " + o.reason);
    }
    private static void cameraHeightFromFieldTrace() {
        PollenHiveSettings.Values defaults=new PollenHiveSettings.Values(PollenHiveSettings.CAMERA,false);
        IsHiveReadyForShot.Mount mount=IsHiveReadyForShot.Mount.from(defaults);
        IsHiveReadyForShot h=new IsHiveReadyForShot(mount,34);
        for(int i=1;i<=30;i++) h.update(new IsHiveReadyForShot.Frame(i,
                tag(34,52.73,-3,60,mount),tag(35,52.73,0,60,mount),
                tag(36,52.24,3,60,mount),tag(37,53.14,6,60,mount)),i*.02);
        check(h.isHiveReadyForShot(.6),"The measured primary return heights fit the revised UP center");
        for(int i=31;i<=60;i++) h.update(new IsHiveReadyForShot.Frame(i,tag(34,35.6,0,60,mount)),i*.02);
        check(h.state(1.2)==IsHiveReadyForShot.State.DOWN&&!h.isHiveReadyForShot(1.2),"DOWN remains distinct with the new UP center");
    }
    private static void practice() {
        for(boolean secondary:new boolean[]{false,true}) {
            PollenHiveRoute c=new PollenHiveRoute(secondary); c.v[0]=2;
            PollenHiveAuto a=new PollenHiveAuto(c,hive(),PollenHiveAuto.Run.SHOT_ONLY); a.start(0);
            PollenHiveAuto.Output o=null; boolean fed=false;
            for(int i=1;i<=1351;i++) {
                double t=i*.02;
                double velocity=secondary||t>=.8?1180:1000;
                o=a.tick(t,secondary&&t>1?frame(i,true):null,0,0,0,velocity,true);
                check(o.forward==0&&o.strafe==0&&o.turn==0,"Shot-only never drives");
                check(!o.phase.startsWith("PARK")&&!o.phase.startsWith("GARDEN"),"Shot-only never enters route");
                if(secondary&&t<=1) check(!o.feed,"Secondary shot-only waits for physical UP");
                if(!secondary&&t<.8) check(!o.feed,"Primary shot-only still waits for measured launcher speed");
                if(!secondary&&i==40) check(o.feed,"Primary shot-only starts as soon as speed is ready without camera");
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
                check(o.phase.startsWith(run==PollenHiveAuto.Run.MATCH?"OUTBOUND":"GARDEN"),"No flywheel speed or minimum-time gate before garden");
                check(o.launcher==0&&o.agitator==0&&o.intake==0,"Departure transition switches all mechanisms off");
            }
        }
        PollenHiveRoute c=new PollenHiveRoute(false); c.v[0]=2;
        PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
        PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot();
        robot.leftLimit=PollenFieldGeometry.END_WALL_INCHES-9+PollenFieldGeometry.PRIMARY_START_X_INCHES; // side wall, v43 start
        boolean fed=false,collected=false; String previous=""; double firstEnd=Double.NaN;
        for(int i=1;i<=1400;i++) {
            // Hold measured flywheel speed high for the WHOLE run, even with its command off.
            PollenHiveAuto.Output o=a.tick(i*.02,frame(i,true),robot.x,robot.y,robot.h,1180,true);
            fed|=o.feed;
            if(o.phase.startsWith("OUTBOUND")&&Double.isNaN(firstEnd)) firstEnd=i*.02;
            if(o.phase.startsWith("GARDEN")||o.phase.startsWith("OUTBOUND")) {
                check(clearOutOnly(o,i*.02,firstEnd,false),"Garden powers launcher or agitator only for the post-volley clear-out");
                collected|=o.intake>0;
                if(previous.equals("FIRST")) check(o.forward==0&&o.strafe==0&&o.turn==0&&o.intake==1,
                        "Shot exit starts the clear-out before path motion");
            }
            previous=o.phase;
            robot.step(o.forward,o.strafe,o.turn,false,.02);
            if(o.finished) break;
        }
        check(fed&&collected,"A live first shot leads to collection despite persistent high measured flywheel speed");
    }
    private static PollenHiveAuto.Output oDummy;
    /** Play 2 secondary waits out of the way until the corner-to-park deadline. */
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
        double plannedDeparture=27-c.cornerParkReserve(k.x,k.y);
        for(boolean tips:new boolean[]{true,false}) {
            PollenHiveAuto a=new PollenHiveAuto(c,hive()); a.start(0);
            PollenOdometryTest.SimRobot robot=new PollenOdometryTest.SimRobot();
            PollenHiveAuto.Output o=null; List<String> order=new ArrayList<>();
            double roomAt=Double.NaN,waitAt=Double.NaN,parkAt=Double.NaN,t=0; boolean fed=false;
            for(int i=1;i<=1400;i++) {
                t=i*.02;
                o=a.tick(t,tips?frame(i,true):new IsHiveReadyForShot.Frame(i),robot.x,robot.y,robot.h,o==null?0:o.launcher,true);
                String ph=o.phase.split(":")[0];
                if(order.isEmpty()||!order.get(order.size()-1).equals(ph)) order.add(ph);
                fed|=o.feed;
                if(ph.equals("ROOM")&&Double.isNaN(roomAt)) roomAt=t;
                if(ph.equals("WAIT")&&Double.isNaN(waitAt)) waitAt=t;
                if(ph.equals("PARK")&&Double.isNaN(parkAt)) parkAt=t;
                if(ph.equals("ROOM")||ph.equals("WAIT")||ph.equals("PARK")) check(o.launcher==0&&o.agitator==0&&!o.feed,"No shooting after leaving");
                if(ph.equals("WAIT")) check(o.forward==0&&o.strafe==0&&o.turn==0&&o.intake==0,"Sits still in the corner with intake off");
                robot.step(o.forward,o.strafe,o.turn,false,.02);
                if(ph.equals("WAIT")) check(Math.hypot(robot.x-k.x,robot.y-k.y)<=.75,"Waits at the corner");
                if(o.finished) break;
            }
            System.out.printf(Locale.US,"Hive make room tips=%s: room %.2f s, corner %.2f s, park %.2f s, done %.2f s %s%n",tips,roomAt,waitAt,parkAt,t,order);
            check(fed==tips,"Shoots only when UP");
            check(order.equals(Arrays.asList("FIRST","ROOM","WAIT","PARK","DONE"))
                    || (!tips&&order.equals(Arrays.asList("FIRST","ROOM","PARK","DONE"))),
                    "Wait when arriving early, then park: "+order);
            check(Math.abs(parkAt-plannedDeparture)<.1,"Park starts on the projected reserve plus margin: "+parkAt);
            check(o.finished&&o.reason.equals("Finished departure")&&t<27,"Parks before the 27 s stop: "+o.reason+" at "+t);
            check(Math.hypot(robot.x-park[0].x,robot.y-park[0].y)<=.75,"Ends in the outer slot");
            if(tips) check(roomAt<10,"Heads to the corner right after the shot: "+roomAt);
        }
        c.v[c.values.index("parkMargin")]=-0.5; rejected=false;
        try { c.validate(); } catch(IllegalArgumentException ex) { rejected=true; }
        check(rejected,"Invalid park margin is rejected at INIT");
        c.v[c.values.index("parkMargin")]=1; c.v[PollenHiveRoute.ROOM_LEFT]=-140; rejected=false;
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

    /** Match 2 (v39): INIT order for every hive mode. Side/Alliance first, checks next, Execution
     * hidden, every other row reachable exactly once, never-used rows at the end. */
    private static void menuOrder() {
        PollenHiveAuto.Run M=PollenHiveAuto.Run.MATCH, S=PollenHiveAuto.Run.SHOT_ONLY, G=PollenHiveAuto.Run.GARDEN_RETURN;
        Object[][] modes={{false,false,false,false,M},{true,false,false,false,M},{false,false,true,false,M},
            {false,false,false,true,M},{true,true,false,false,M},{false,false,false,false,S},{true,false,false,false,S},
            {false,false,false,false,G}};
        for(Object[] m:modes) {
            boolean sec=(Boolean)m[0], room=(Boolean)m[1];
            PollenHiveAuto.Run run=(PollenHiveAuto.Run)m[4];
            PollenHiveRoute r=new PollenHiveRoute(sec,room,(Boolean)m[2],(Boolean)m[3]);
            int[] o=r.menuOrder(run);
            String mode=run+(sec?" secondary":" primary")+(room?" make-room":"")+(m[2].equals(true)?" fast":"")+(m[3].equals(true)?" angled":"");
            int[] hidden=java.util.stream.IntStream.range(0,r.values.rows.length).filter(i -> {
                String key=r.values.rows[i].key;
                return key.equals("execution")||key.equals("collectPower")||key.equals("feedIntake")
                        ||key.equals("matchFeedIntake")||key.equals("collection.intake");
            }).toArray();
            check(o.length==r.values.rows.length-hidden.length,mode+": legacy rows hidden");
            java.util.Set<Integer> seen=new java.util.HashSet<>();
            for(int i:o) check(seen.add(i),mode+": row listed once");
            for(int i:hidden) check(!seen.contains(i),mode+": legacy row hidden: "+r.values.rows[i].key);
            check(r.values.rows[o[0]].key.equals("side"),mode+": side / Alliance first");
            String second=r.values.rows[o[1]].key;
            String want=!sec?"startGapIn":"launcher"; // v43: primary placement first
            check(second.equals(want),mode+": first check row is "+second);
            int legacy=r.values.index("roomParkSecondsLeft");
            int at=-1; for(int i=0;i<o.length;i++) if(o[i]==legacy) at=i;
            check(room||at>o.length-25,mode+": unused rows at the end");
        }
        PollenHiveRoute primary=new PollenHiveRoute(false);
        int[] o=primary.menuOrder(M);
        String[] want={"side","startGapIn","startDelay","launcher","matchTravelPower","matchFirstFeed","matchSecondFeed",
            "matchGardenPasses","parkMargin","clearOut"};
        for(int i=0;i<want.length;i++) check(primary.values.rows[o[i]].key.equals(want[i]),"primary row "+(i+1)+" is "+want[i]);
        check(primary.values.rows[o[want.length]].key.startsWith("collection."),"garden tuning follows the checks");
    }

    public static void main(String[] args) {
        menuOrder();
        secondaryFlowerOff();
        matchAllianceSetup();
        classifier(); missesVsContradictions(); autonomousTagGate(); unsettledReturnParks(); returnHeadingCorrection(2.07); returnHeadingCorrection(-2.07);
        calibration(); sides(); settings(); routes(); collectionTuning(); secondaryGates(); clearOutAfterVolley(); fastCycle(); returnNudge(); pacedFeed();
        double fullPower=primaryCycle(false,false,1);
        double priorPower=primaryCycle(false,false,.6);
        check(fullPower<priorPower,"Full-power transit improves simulated cycle time");
        primaryCycle(false,true,1); primaryCycle(true,false,1); secondaryFlower(true); secondaryFlower(false);
        slowMatchPark(false, .30); slowMatchPark(true, .20); slowMatchPark(false, .10);
        lateCollectionAndPark(); parkFromCollectionLane(); parkFinishesReturnLegs(); squaresUpInPlace(); angledPrimary(); angledPrimaryReturnOff(.65,true); angledPrimaryReturnOff(1.3,false); allianceForTeleOp(); angledSecondary(true); angledSecondary(false); angledSecondaryMissesPreload(false); angledSecondaryMissesPreload(true); cameraHeightFromFieldTrace(); motionFaults(); practice(); noCoastGate(); makeRoom();
        System.out.println("PASS PollenHiveTest: "+checks+" checks");
    }
}
