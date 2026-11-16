package org.firstinspires.ftc.teamcode.pollen;

public final class PollenCameraDriveControlTest {
    private static int checks;
    private static void powers(double[] actual, double... expected) {
        checks++;
        for (int i = 0; i < 4; i++) if (Math.abs(actual[i] - expected[i]) > 1e-9)
            throw new AssertionError("Wrong wheel " + i + ": " + actual[i] + " != " + expected[i]);
    }
    public static void main(String[] args) {
        PollenCameraDriveControl c = new PollenCameraDriveControl();
        c.reset();
        powers(c.update(true, false, 1, 0, 0), 0, 0, 0, 0); // L1 held across START
        powers(c.update(false, false, 1, 1, 1), 0, 0, 0, 0);
        powers(c.update(true, false, 1, 0, 0), .3, .3, .3, .3);
        powers(c.update(true, false, 0, 1, 0), .3, -.3, -.3, .3);
        powers(c.update(true, false, 0, 0, 1), .3, -.3, .3, -.3);
        powers(c.update(true, false, 1, 1, 1), .3, -.1, .1, .1); // normalize BEFORE cap
        powers(c.update(true, true, 1, 1, 1), .15, -.05, .05, .05);
        powers(c.update(true, false, .04, -.04, .01), 0, 0, 0, 0);
        powers(c.update(true, false, Double.NaN, 0, 0), 0, 0, 0, 0);
        powers(c.update(true, false, 0, Double.POSITIVE_INFINITY, 0), 0, 0, 0, 0);
        powers(c.update(false, true, 1, 1, 1), 0, 0, 0, 0); // release stops immediately
        c.reset();
        powers(c.update(true, true, -1, 0, 0), 0, 0, 0, 0); // new run needs release again
        System.out.println("PASS PollenCameraDriveControlTest: " + checks + " checks");
    }
}
