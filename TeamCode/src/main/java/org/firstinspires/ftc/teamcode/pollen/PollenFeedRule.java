package org.firstinspires.ftc.teamcode.pollen;

/**
 * THE FEED RULE: when may balls be fed into the launcher?
 *
 * There is one rule, and PollenBot TeleOp, every autonomous shot and LauncherTest all use it
 * from here, so they cannot drift apart.
 *
 * Feeding may START when either of these is true, whichever happens first:
 *   1. UP TO SPEED: the launcher's encoder reads at least 96% of the target speed
 *      (PollenConfig.LAUNCHER_READY_FRACTION), or
 *   2. THE SPIN-UP TIME IS OVER: the launcher has been on for the whole spin-up time. This is
 *      the backup. With a wrong encoder reading (an unplugged cable reads 0) rule 1 never comes
 *      true, and on October 24, before the backup existed, the robot would not shoot at all.
 *
 * Once feeding has started it keeps going until the shot is over (R1 released in TeleOp, the
 * feed time used up in autonomous). Each ball slows the wheel a little, and that dip must not
 * switch the feed off in the middle of a volley. Remembering "feeding has started" is the
 * caller's job: see launch() in the TeleOp and shootStep() in PollenSequence.
 *
 * Pure Java, no hardware, so it is tested on a computer.
 */
public final class PollenFeedRule {
    private PollenFeedRule() { }

    /** Rule 1. Speeds are in encoder ticks per second. A reading that is not a number is not up to speed. */
    public static boolean upToSpeed(double measuredVelocity, double targetVelocity) {
        return measuredVelocity >= targetVelocity * PollenConfig.LAUNCHER_READY_FRACTION;
    }

    /** Rule 2, the backup. */
    public static boolean spinUpTimeIsOver(double secondsSinceLauncherOn, double spinupSeconds) {
        return secondsSinceLauncherOn >= spinupSeconds;
    }

    /** Rule 1 or rule 2. */
    public static boolean mayStartFeeding(double measuredVelocity, double targetVelocity,
                                          double secondsSinceLauncherOn, double spinupSeconds) {
        return upToSpeed(measuredVelocity, targetVelocity)
                || spinUpTimeIsOver(secondsSinceLauncherOn, spinupSeconds);
    }
}
