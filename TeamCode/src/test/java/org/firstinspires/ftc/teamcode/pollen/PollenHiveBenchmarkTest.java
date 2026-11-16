package org.firstinspires.ftc.teamcode.pollen;

/** Timed switches and human labels: no hardware needed. */
public final class PollenHiveBenchmarkTest {
    private static int checks;
    private static void check(boolean b, String why) { checks++; if (!b) throw new AssertionError(why); }
    public static void main(String[] args) {
        PollenHiveBenchmarkPlan p = new PollenHiveBenchmarkPlan(2, 4, 2, 20);
        p.confirm("DOWN", 0); check(p.phase == PollenHiveBenchmarkPlan.Phase.WAIT, "wrong label must not start");
        double now = 0;
        int trials = 0;
        for (int block = 0; block < 4; block++) {
            check(p.phase == PollenHiveBenchmarkPlan.Phase.WAIT, "must wait for every human label");
            check(p.expected().equals(block % 2 == 0 ? "UP" : "DOWN"), "labels alternate");
            p.confirm(p.expected(), now);
            for (int slot = 0; slot < 3; slot++) {
                check(p.phase == PollenHiveBenchmarkPlan.Phase.SWITCH, "switch every trial");
                check(p.pipeline() == (block < 2 ? 2 + slot : 4 - slot), "counterbalanced order");
                check(p.trial() == trials++, "unique trial id");
                p.switched(true, now);
                p.tick(now + 1.99); check(p.phase == PollenHiveBenchmarkPlan.Phase.SETTLE, "exclude startup");
                p.tick(now + 2); check(p.phase == PollenHiveBenchmarkPlan.Phase.RECORD, "record after settle");
                p.confirm("DOWN", now + 3); check(p.phase == PollenHiveBenchmarkPlan.Phase.RECORD, "ignore labels midtrial");
                p.tick(now + 21.99); check(p.phase == PollenHiveBenchmarkPlan.Phase.RECORD, "full measurement time");
                now += 22;
                p.tick(now);
            }
        }
        check(p.phase == PollenHiveBenchmarkPlan.Phase.DONE, "all trials finish");
        p = new PollenHiveBenchmarkPlan(9, 9, 1, 5);
        p.confirm("UP", 0); p.switched(false, 1); p.tick(99);
        check(p.phase == PollenHiveBenchmarkPlan.Phase.ERROR, "failed switch cannot record");
        // A slow camera switch: the 2 s settle only counts once frames come from the new pipeline.
        p = new PollenHiveBenchmarkPlan(0, 1, 1, 5);
        check(p.trials() == 4, "2 pipelines x UP and DOWN");
        check(PollenHiveBenchmarkPlan.plannedSeconds(0, 4, 2, 20) == 440, "five pipelines, two passes = 7 min 20 s");
        p.confirm("UP", 0); p.switched(true, 0);
        for (double t = 0; t < 3.5; t += .02) { p.settle(false, t); p.tick(t); }
        check(p.phase == PollenHiveBenchmarkPlan.Phase.SETTLE, "no recording while the camera is still on the old pipeline");
        p.settle(true, 3.5); p.tick(3.5); p.settle(true, 5.4); p.tick(5.4);
        check(p.phase == PollenHiveBenchmarkPlan.Phase.SETTLE, "full settle time on the new pipeline");
        p.settle(true, 5.5); p.tick(5.5);
        check(p.phase == PollenHiveBenchmarkPlan.Phase.RECORD, "record 2 s after the new pipeline's first frame");
        p.settle(false, 6); check(p.since == 5.5, "settle never moves a recording's start");
        p.tick(10.5); check(p.phase == PollenHiveBenchmarkPlan.Phase.SWITCH, "next pipeline");
        // A pipeline that never sends a frame is still recorded (as zero frames), not waited on forever.
        p.switched(true, 11);
        for (double t = 11; t < 22.9; t += .02) { p.settle(false, t); p.tick(t); }
        check(p.phase == PollenHiveBenchmarkPlan.Phase.SETTLE, "waits up to the switch limit, then the settle time");
        for (double t = 22.9; t < 23.2; t += .02) { p.settle(false, t); p.tick(t); }
        check(p.phase == PollenHiveBenchmarkPlan.Phase.RECORD, "records anyway after the switch limit");
        check(Math.abs(p.since - 23) < .05, "limit + settle = 12 s after the switch");
        int[][] bad = {{-1,0,1,5},{0,10,1,5},{2,1,1,5},{0,0,0,5},{0,0,6,5},{0,0,1,0}};
        for (int[] b : bad) {
            boolean threw = false;
            try { new PollenHiveBenchmarkPlan(b[0], b[1], b[2], b[3]); } catch (IllegalArgumentException ex) { threw = true; }
            check(threw, "invalid plan rejected");
        }
        System.out.println("Hive benchmark plan: " + checks + " checks passed");
    }
}
