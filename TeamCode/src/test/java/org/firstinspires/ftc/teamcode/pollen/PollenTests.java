package org.firstinspires.ftc.teamcode.pollen;

import org.junit.Test;

/**
 * The PollenBot computer tests, runnable from Android Studio on any laptop: open this file and
 * click the green arrow next to {@code class PollenTests}, or right-click it and choose Run.
 * No robot is needed. From a terminal: {@code gradlew :TeamCode:testDebugUnitTest}.
 *
 * Each test below runs one test program. A program throws an AssertionError that says what
 * went wrong as soon as one of its checks fails, so a red test's message is the place to start.
 * These are the same programs tools/test-pollen-sequence.sh runs (that script also runs the
 * Python field-route checker, for mentors).
 */
public class PollenTests {
    private static final String[] NO_ARGS = {};

    /** Autonomous step runner: timing, time budget, stops, odometry moves, the robot log. */
    @Test public void sequence() { PollenSequenceTest.main(NO_ARGS); }

    /** Park-only INIT menu. */
    @Test public void parkMenu() { PollenParkTuningTest.main(NO_ARGS); }

    /** Shoot + Park: the shot, the INIT menu and its time budget. */
    @Test public void shootFirst() { PollenShootFirstTest.main(NO_ARGS); }

    /** Supporting shooter and the field-tuned routes. */
    @Test public void supportingShooter() { PollenMatSupportTest.main(NO_ARGS); }

    /** PollenBot TeleOp's INIT menu. */
    @Test public void teleOpMenu() { PollenTeleOpTuningTest.main(NO_ARGS); }

    /** Autonomous with slow and irregular loops. */
    @Test public void loopTiming() { PollenLoopTimingTest.main(NO_ARGS); }

    /** Odometry autos against a simulated robot (a stand-in, not the real robot). */
    @Test public void odometry() { PollenOdometryTest.main(NO_ARGS); }
}
