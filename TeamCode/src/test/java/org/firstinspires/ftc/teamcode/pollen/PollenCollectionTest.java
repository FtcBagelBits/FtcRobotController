package org.firstinspires.ftc.teamcode.pollen;

import java.util.ArrayList;
import java.util.List;
import static org.firstinspires.ftc.teamcode.pollen.PollenCollectionSequence.Kind.*;

/** Simulate the actual wheel mixer, a hard stop at the flower and a nonzero match entry pose. */
public final class PollenCollectionTest {
    private static void check(boolean yes, String why) { if (!yes) throw new AssertionError(why); }
    private static void near(double actual, double expected, String why) {
        check(Math.abs(actual-expected) < 1.5, why + ": " + actual + " vs " + expected + " cm");
    }
    private static void edit(PollenHiveSettings.Values s, String key, double value) { s.v[s.index(key)] = value; }

    private static final class Trial {
        double forward, left, furthest, seat, endLeft, endRight;
        final List<String> phases = new ArrayList<>();
        Trial(PollenCollectionSequence.Kind kind, PollenHiveSettings.Values s, double heading) {
            final double x0 = 13, y0 = -7, dt = .02, hardStop = 19;
            double cos = Math.cos(heading), sin = Math.sin(heading);
            double[] entry = PollenFacing.routePose(x0,y0,heading);
            PollenCollectionSequence seq = new PollenCollectionSequence(kind, s, entry[0],entry[1],heading);
            String last = "";
            for (int i = 0; i < 15000; i++) {
                double[] p = PollenFacing.routePose(x0 + (cos*forward - sin*left)/2.54,
                        y0 + (sin*forward + cos*left)/2.54, heading);
                PollenCollectionSequence.Output o = seq.tick(i*dt,p[0],p[1],p[2],true);
                check(o.fault == null, "Fault in " + last + ": " + o.fault);
                if (!last.equals(o.phase)) {
                    phases.add(o.phase);
                    if (o.phase.equals("Left 1")) seat = forward;
                    if (last.equals("Left 1")) endLeft = left;
                    if (last.equals("Right 1")) endRight = left;
                    last = o.phase;
                }
                if (o.done) {
                    off(o);
                    off(seq.tick(400,0,0,0,false));
                    return;
                }
                check(o.intake == (kind == FLOWER ? 1 : s.get("intake")), "Intake dropped in " + o.phase);
                boolean cleanup = o.phase.equals("CLEANUP") || o.phase.startsWith("Ingest forward")
                        || o.phase.startsWith("Back out") || o.phase.equals("Finish ingesting");
                check(o.drainerDown == (kind == FLOWER && !cleanup), "Drainer state in " + o.phase);
                double[] w = PollenFacing.wheelPowers(o.forward,o.strafe,o.turn);
                forward += (w[0]+w[1]+w[2]+w[3])/4 * 95*dt;
                left += (-w[0]+w[1]+w[2]-w[3])/4 * 95*dt;
                if (kind == FLOWER) forward = Math.min(forward, hardStop);
                furthest = Math.max(furthest,forward);
            }
            throw new AssertionError("Never finished: " + last);
        }
    }
    private static void off(PollenCollectionSequence.Output o) {
        check(o.forward == 0 && o.strafe == 0 && o.turn == 0 && o.intake == 0 && !o.drainerDown, "Outputs live after stop");
    }
    private static void expectInvalid(PollenCollectionSequence.Kind kind, PollenHiveSettings.Values s) {
        try { PollenCollectionSequence.validate(kind,s); }
        catch (IllegalArgumentException expected) { return; }
        throw new AssertionError("Invalid profile accepted");
    }

    public static void main(String[] args) {
        PollenHiveSettings.Values g = PollenCollectionSequence.settings(GARDEN);
        edit(g,"shift",4);
        Trial garden = new Trial(GARDEN,g,.7);
        check(garden.phases.contains("Inward pass 3") && !garden.phases.contains("Inward pass 4"), "Garden pass count");
        near(garden.furthest,12.70,"Approach depth");
        near(garden.forward,0,"Garden exit");
        near(garden.left,8,"Two lane steps");
        edit(g,"passes",1);
        edit(g,"exit",0);
        edit(g,"hold",0);
        Trial one = new Trial(GARDEN,g,0);
        check(one.phases.size() == 2 && one.phases.get(0).equals("Inward pass 1"), "Minimum garden plan");
        near(one.forward,12.70,"Single-pass exit");

        PollenHiveSettings.Values f = PollenCollectionSequence.settings(FLOWER);
        Trial flower = new Trial(FLOWER,f,-1.4);
        near(flower.seat,19,"Seated at actual wall");
        near(flower.endLeft,3,"First left swing");
        near(flower.endRight,-3,"First right swing");
        near(flower.forward,flower.seat-18,"Flower exit relative to actual contact");
        near(flower.left,0,"Centered on exit");
        check(flower.phases.indexOf("Right 3") < flower.phases.indexOf("Withdraw drainer")
                && flower.phases.contains("Right 3") && !flower.phases.contains("Left 4"), "Three wiggles");
        check(flower.phases.contains("Ingest forward 2") && !flower.phases.contains("Ingest forward 3"), "Two cleanup cycles");
        edit(f,"shakes",0);
        edit(f,"repeats",0);
        edit(f,"hold",0);
        Trial withdraw = new Trial(FLOWER,f,0);
        check(!withdraw.phases.contains("Left 1") && !withdraw.phases.contains("Ingest forward 1")
                && withdraw.phases.contains("Withdraw drainer"), "Withdrawal with zero repeats");

        PollenCollectionSequence seq = new PollenCollectionSequence(FLOWER,f,0,0,0);
        check(seq.tick(0,0,0,0,true).drainerDown,"Lower on START");
        off(seq.stop());
        off(seq.tick(.02,0,0,0,true));
        seq = new PollenCollectionSequence(GARDEN,g,0,0,0);
        seq.tick(0,0,0,0,true);
        PollenCollectionSequence.Output stalled = seq.tick(.4,0,0,0,true);
        check(stalled.fault != null,"Loop stall");
        off(stalled);
        off(seq.tick(.42,0,0,0,true));
        seq = new PollenCollectionSequence(FLOWER,f,0,0,0);
        PollenCollectionSequence.Output lost = seq.tick(0,0,0,0,false);
        check(lost.fault != null,"Pinpoint fault");
        off(lost);
        seq = new PollenCollectionSequence(GARDEN,g,0,0,0);
        PollenCollectionSequence.Output jam = null;
        for (int i=0; i<100; i++) jam = seq.tick(i*.02,0,0,0,true);
        check(jam.fault != null && jam.fault.contains("no odometry progress"), "Stalled approach");
        off(jam);

        edit(f,"repeats",1);
        edit(f,"stroke",f.get("back"));
        expectInvalid(FLOWER,f);
        edit(g,"passes",1.5);
        expectInvalid(GARDEN,g);
        edit(g,"passes",1);
        edit(g,"clear",g.get("gap"));
        expectInvalid(GARDEN,g);
        System.out.println("Collection practice: garden, flower, rotated entry, stop/fault and profile checks passed");
    }
}
