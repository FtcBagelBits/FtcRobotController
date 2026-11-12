package org.firstinspires.ftc.teamcode.pollen;

import java.util.*;

/** Computer-only regression tests. Synthetic observations and a simple drivetrain are NOT field validation. */
public final class PollenHiveTest {
    private static int checks;
    private static void check(boolean ok,String why) { checks++; if(!ok) throw new AssertionError(why); }
    private static IsHiveReadyForShot.Profile profile() {
        return new IsHiveReadyForShot.Profile(0,34,
            new IsHiveReadyForShot.Box(new double[]{-.1,-.6,1.1},new double[]{.1,-.4,1.3}),
            new IsHiveReadyForShot.Box(new double[]{-.1,-.2,1.1},new double[]{.1,0,1.3}));
    }
    private static IsHiveReadyForShot.Frame frame(double stamp,boolean up) {
        return new IsHiveReadyForShot.Frame(stamp,0,up?-.5:-.1,1.2,true);
    }
    private static void classifier() {
        IsHiveReadyForShot h=new IsHiveReadyForShot(profile());
        check(!h.isHiveReadyForShot(0),"Starts unknown");
        for(int i=1;i<=12;i++) h.update(frame(i,true),i*.02);
        check(h.isHiveReadyForShot(.24),"Stable UP grants permission");
        check(!h.isHiveReadyForShot(.45),"Getter expires without more update calls");
        h.update(frame(12,true),.46); check(!h.isHiveReadyForShot(.46),"Frozen camera cannot rearm");
        for(int i=13;i<=26;i++) h.update(frame(i,true),.46+(i-12)*.02);
        check(h.isHiveReadyForShot(.74),"Distinct frames recover");
        h.update(frame(27,false),.76); check(!h.isHiveReadyForShot(.76),"One contradiction revokes immediately");
        for(int i=28;i<=40;i++) h.update(frame(i,false),.76+(i-27)*.02);
        check(h.state(1.02)==IsHiveReadyForShot.State.DOWN,"DOWN confirms independently");
        h.update(null,1.04); check(h.state(1.04)==IsHiveReadyForShot.State.UNKNOWN,"Missing is UNKNOWN");
        h.update(frame(1,true),1.06); check(!h.isHiveReadyForShot(1.06),"Camera reboot requires reconfirmation");
        h.update(new IsHiveReadyForShot.Frame(2,0,-.5,0,true),1.08);
        check(!h.isHiveReadyForShot(1.08),"Missing pose zeros rejected");
        for(int i=3;i<20;i++) h.update(new IsHiveReadyForShot.Frame(i,2,-.5,1.2,true),1.08+i*.02);
        check(!h.isHiveReadyForShot(1.46),"Wrong viewpoint rejected");
        h=new IsHiveReadyForShot(new IsHiveReadyForShot.Profile(0,34,profile().up,null));
        for(int i=1;i<=20;i++) h.update(frame(i,false),i*.02);
        check(h.state(.4)==IsHiveReadyForShot.State.UNKNOWN,"UP-only does not invent DOWN");
        IsHiveReadyForShot.Capture c=new IsHiveReadyForShot.Capture();
        for(int i=0;i<100;i++) c.add(frame(1,true));
        check(c.count==1,"Capture deduplicates frames");
        boolean rejected=false; try { c.finish(); } catch(IllegalArgumentException ex) { rejected=true; }
        check(rejected,"Insufficient capture rejected");
        for(int i=2;i<=41;i++) c.add(frame(i,true));
        check(c.finish().contains(frame(42,true)),"Capture builds envelope");
        rejected=false; try { new IsHiveReadyForShot.Profile(0,34,profile().up,profile().up); }
        catch(IllegalArgumentException ex) { rejected=true; }
        check(rejected,"Overlapping states rejected");
    }
    private static void routes() {
        PollenHiveRoute c=new PollenHiveRoute(false); c.validate();
        System.out.printf(Locale.US,"Hive CAD defaults: garden %.2f s, park %.2f s (estimates only)%n",PollenHiveRoute.planned(c.garden()),PollenHiveRoute.planned(c.park()));
        PollenHiveRoute.Node[] nodes=c.garden();
        check(Math.abs(nodes[3].h+Math.PI/2)<1e-9,"Transit turns front toward garden");
        check(nodes[6].collect&&nodes[6].h==0,"Pickup faces front toward garden wall");
        check(nodes[nodes.length-1].x==0&&nodes[nodes.length-1].y==0&&nodes[nodes.length-1].h==0,"Return restores shoot pose");
        check(Math.abs(PollenFieldGeometry.RED_HIVE_X_INCHES-nodes[6].y+60.585)<1e-6,"Pickup center derived from side-wall clearance");
        c.v[1]=2; Properties p=c.properties(); c.load(p); check(c.v[1]==0,"Saved settings never reload LIVE");
        c.v[13]=0; boolean rejected=false; try { c.validate(); } catch(IllegalArgumentException ex) { rejected=true; }
        check(rejected,"Turn with no end-wall clearance rejected");
        new PollenHiveRoute(true).validate();
    }
    private static void secondaryGates() {
        PollenHiveRoute c=new PollenHiveRoute(true); c.v[1]=2; c.v[3]=10; c.v[21]=0;
        IsHiveReadyForShot h=new IsHiveReadyForShot(profile()); PollenHiveAuto a=new PollenHiveAuto(c,h); a.start(0);
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
        PollenHiveRoute c=new PollenHiveRoute(false); c.v[1]=driveOnly?1:2;
        IsHiveReadyForShot h=new IsHiveReadyForShot(profile()); PollenHiveAuto a=new PollenHiveAuto(c,h); a.start(0);
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
        PollenHiveRoute c=new PollenHiveRoute(false); c.v[1]=2;
        PollenHiveAuto a=new PollenHiveAuto(c,new IsHiveReadyForShot(profile())); a.start(0);
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
        a=new PollenHiveAuto(c,new IsHiveReadyForShot(profile())); a.start(0);
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
        PollenHiveAuto a=new PollenHiveAuto(c,new IsHiveReadyForShot(profile())); a.start(0);
        for(int i=1;i<=1351;i++) oDummy=a.tick(i*.02,null,0,0,0,0,true);
        check(oDummy.finished,"Deadline stops even with no observation");
    }
    private static PollenHiveAuto.Output oDummy;
    public static void main(String[] args) {
        classifier(); routes(); secondaryGates(); primaryCycle(false,false); primaryCycle(false,true); primaryCycle(true,false); slowLauncherBudget(); motionFaults();
        System.out.println("PASS PollenHiveTest: "+checks+" checks");
    }
}
