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
        int[][] bad = {{-1,0,1,5},{0,10,1,5},{2,1,1,5},{0,0,0,5},{0,0,6,5},{0,0,1,0}};
        for (int[] b : bad) {
            boolean threw = false;
            try { new PollenHiveBenchmarkPlan(b[0], b[1], b[2], b[3]); } catch (IllegalArgumentException ex) { threw = true; }
            check(threw, "invalid plan rejected");
        }
        System.out.println("Hive benchmark plan: " + checks + " checks passed");
    }
}
