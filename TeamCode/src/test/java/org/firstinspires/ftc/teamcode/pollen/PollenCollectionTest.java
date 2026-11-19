package org.firstinspires.ftc.teamcode.pollen;

import java.util.ArrayList;
import java.util.List;
import java.util.Properties;
import static org.firstinspires.ftc.teamcode.pollen.PollenCollectionSequence.Kind.*;

/** Simulate the wheel mixer and hard stops from a nonzero match entry pose. */
public final class PollenCollectionTest {
    private static void check(boolean yes, String why) { if (!yes) throw new AssertionError(why); }
    private static void near(double actual, double expected, String why) {
        check(Math.abs(actual-expected) < 1.5, why + ": " + actual + " vs " + expected + " cm");
    }
    private static void edit(PollenHiveSettings.Values s, String key, double value) { s.v[s.index(key)] = value; }

    private static final class Trial {
        double forward, left, seat, endLeft, endRight, seconds;
        String fault;
        final List<String> phases = new ArrayList<>();
        final List<String> limited = new ArrayList<>();
        final List<Double> backoffPositions = new ArrayList<>();
        Trial(PollenCollectionSequence.Kind kind, PollenHiveSettings.Values s, double heading) {
            this(kind, s, heading, 23, false);
        }
        Trial(PollenCollectionSequence.Kind kind, PollenHiveSettings.Values s, double heading,
              double hardStop, boolean bindDrainer) {
            this(kind, s, heading, hardStop, bindDrainer, Double.POSITIVE_INFINITY);
        }
        Trial(PollenCollectionSequence.Kind kind, PollenHiveSettings.Values s, double heading,
              double hardStop, boolean bindDrainer, double sideWall) {
            final double x0 = 13, y0 = -7, dt = .02;
            double cos = Math.cos(heading), sin = Math.sin(heading);
            double[] entry = PollenFacing.routePose(x0,y0,heading);
            PollenCollectionSequence seq = new PollenCollectionSequence(kind, s, entry[0],entry[1],heading);
            String last = "";
            boolean flowerSeated = false;
            for (int i = 0; i < 15000; i++) {
                double[] p = PollenFacing.routePose(x0 + (cos*forward - sin*left)/2.54,
                        y0 + (sin*forward + cos*left)/2.54, heading);
                PollenCollectionSequence.Output o = seq.tick(i*dt,p[0],p[1],p[2],true);
                if (o.fault != null) {
                    fault = o.fault;
                    off(o);
                    return;
                }
                if (o.contactLimit != null) {
                    check(o.forward == 0 && o.strafe == 0 && o.turn == 0, "Drive live at contact limit");
                    check(o.contactLimit.contains("no_progress_s="), "Missing contact diagnostics");
                    limited.add(o.phase);
                    flowerSeated = o.phase.equals("Approach flower") || o.phase.equals("Seat drainer")
                            || flowerSeated;
                }
                if (flowerSeated) check(!o.phase.equals("Seat drainer") || o.forward == 0,
                        "Continued pushing after flower contact");
                if (!last.equals(o.phase)) {
                    phases.add(o.phase);
                    if (o.phase.equals("Left 1")) seat = forward;
                    if (last.equals("Left 1")) endLeft = left;
                    if (last.equals("Right 1")) endRight = left;
                    if (last.startsWith("Back off ")) backoffPositions.add(forward);
                    last = o.phase;
                }
                if (o.done) {
                    seconds = i * dt;
                    off(o);
                    off(seq.tick(400,0,0,0,false));
                    return;
                }
                check(o.intake == (kind == FLOWER ? 1 : s.get("intake")), "Intake dropped in " + o.phase);
                check(o.drainerDown == (kind == FLOWER), "Drainer state in " + o.phase);
                double[] w = PollenFacing.wheelPowers(o.forward,o.strafe,o.turn);
                forward += (w[0]+w[1]+w[2]+w[3])/4 * 95*dt;
                if (!bindDrainer || !o.phase.equals("Center drainer"))
                    left += (-w[0]+w[1]+w[2]-w[3])/4 * 95*dt;
                if (bindDrainer && (o.phase.startsWith("Left ") || o.phase.startsWith("Right ")))
                    left = Math.max(-1.5, Math.min(left, 1.5));
                forward = Math.min(forward, hardStop);
                left = sideWall >= 0 ? Math.min(left, sideWall) : Math.max(left, sideWall);
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
        Properties oldProfile = new Properties();
        oldProfile.setProperty("sideLimit", "2.54");
        oldProfile.setProperty("intake", "1.0");
        check(g.load(oldProfile).contains("sideLimit") && g.get("intake") == 1,
                "Ignore the old signed direction while retaining intake power");
        Trial garden = new Trial(GARDEN,g,.7,38,false,-24);
        check(garden.fault == null && garden.limited.equals(java.util.Arrays.asList(
                "Find right wall", "Inward pass 1", "Inward pass 2")), "Garden wall order and repeats");
        near(garden.forward,38,"Finish past the old search limit against garden wall");
        near(garden.left,-24,"Finish past the old side search limit against right wall");
        check(garden.backoffPositions.size() == 1, "One backoff gives two total passes");
        for (double p : garden.backoffPositions) near(p,38-7.62,"Three-inch backoff");

        PollenCollectionSequence againstWall = new PollenCollectionSequence(GARDEN,g,0,0,0);
        againstWall.tick(0,0,0,0,true);
        boolean startedInward = false;
        for (int i = 1; i <= 50; i++) {
            PollenCollectionSequence.Output o = againstWall.tick(i*.02,0,0,Math.toRadians(2.41),true);
            check(o.fault == null, "Wall-induced heading shift must not fault: " + o.fault);
            startedInward |= o.phase.equals("Inward pass 1");
        }
        check(startedInward, "Wall stall at 2.41 degrees advances to the garden");

        edit(g,"repeats",1);
        edit(g,"backoffIn",5);
        Trial oneRepeat = new Trial(GARDEN,g,.7,18,false,-4);
        check(oneRepeat.fault == null && oneRepeat.backoffPositions.size() == 1, "One repeat");
        near(oneRepeat.left,-4,"Right side wall");
        near(oneRepeat.backoffPositions.get(0),18-12.7,"Adjusted five-inch backoff");

        edit(g,"repeats",0);
        Trial one = new Trial(GARDEN,g,0,18,false,-4);
        check(one.fault == null && one.limited.size() == 2 && one.backoffPositions.isEmpty(), "Zero repeats");
        Trial distant = new Trial(GARDEN,g,.7,250,false,-4);
        check(distant.fault == null && distant.seconds > 12 && distant.limited.size() == 2,
                "Keep advancing after the old time limit until a wall blocks motion");
        near(distant.forward,250,"Distant wall contact");

        PollenHiveSettings.Values f = PollenCollectionSequence.settings(FLOWER);
        Trial flower = new Trial(FLOWER,f,-1.4);
        check(flower.fault == null && flower.phases.contains("Finish ingesting"), "Flower runs through final intake dwell");
        near(flower.seat,23,"Seated at actual wall");
        near(flower.endLeft,3.5,"First left swing");
        near(flower.endRight,-3.5,"First right swing");
        near(flower.forward,flower.seat-18,"Flower exit relative to actual contact");
        near(flower.left,0,"Centered on exit");
        check(flower.phases.indexOf("Right 3") < flower.phases.indexOf("Withdraw drainer")
                && flower.phases.contains("Right 3") && !flower.phases.contains("Left 4"), "Three wiggles");
        check(flower.phases.contains("Ingest forward 2") && !flower.phases.contains("Ingest forward 3"), "Two cleanup cycles");
        check(flower.limited.equals(java.util.Collections.singletonList("Seat drainer")), "Normal flower run needs seat contact");
        Trial early = new Trial(FLOWER,f,-1.4,8,false);
        check(early.limited.equals(java.util.Collections.singletonList("Approach flower"))
                && !early.phases.contains("Seat drainer") && early.phases.contains("Right 3"),
                "Blocked approach should skip the final push and drain");
        near(early.seat,8,"Shake starts at early contact");
        near(early.forward,8-18,"Withdraw from early contact");
        Trial deeper = new Trial(FLOWER,f,-1.4,45,false);
        check(deeper.fault == null && deeper.limited.equals(java.util.Collections.singletonList("Seat drainer")),
                "Seating continues past the nominal depth to reach the flower");
        near(deeper.seat,45,"Swing from deeper contact");
        Trial bound = new Trial(FLOWER,f,-1.4,23,true);
        check(bound.limited.size() == 8 && bound.limited.contains("Center drainer")
                && bound.phases.contains("Ingest forward 2") && bound.seconds < 15, "Bound swings and center should withdraw");
        edit(f,"amplitude",5);
        edit(f,"sidePower",.15);
        Trial wide = new Trial(FLOWER,f,-1.4);
        check(wide.fault == null && wide.phases.contains("Right 3"), "Tuned lateral swing completes");
        near(wide.endLeft,5,"Adjusted left swing");
        near(wide.endRight,-5,"Adjusted right swing");
        check(Math.abs(bound.left + 1.5) < .4, "Withdrawal should stay aligned with the constrained drainer");
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
        edit(f,"repeats",1);
        edit(f,"stroke",f.get("back"));
        expectInvalid(FLOWER,f);
        edit(g,"repeats",1.5);
        expectInvalid(GARDEN,g);
        System.out.println("Collection practice: garden, flower, rotated entry, stop/fault and profile checks passed");
    }
}
