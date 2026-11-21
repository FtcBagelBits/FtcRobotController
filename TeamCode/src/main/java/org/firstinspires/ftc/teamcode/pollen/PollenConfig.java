package org.firstinspires.ftc.teamcode.pollen;

/** Shared drive mapping and Option A defaults. INIT controls allow tuning without rebuilding. */
public final class PollenConfig {
    private PollenConfig() { }

    /** Shown in INIT by every Pollen and Odo autonomous mode and by PollenBot TeleOp, and written to
     * the robot log at START. Change it whenever the Pollen code changes. */
    public static final String BUILD = "pollen-hive-v52";
    /** Where Circle (TeleOp) and Beta Primary Shooter - Angled Left aim: this many inches LEFT of
     * the middle of our hive cell (negative = right). The robot shoots a little left and a miss
     * left of center costs more (drive team, 2026-11-19), so the default aims 2 in right. */
    public static final double AIM_TARGET_LEFT_IN = -2.0, AIM_TARGET_STEP_IN = .5, AIM_TARGET_MAX_IN = 12;

    // Hive camera and hive-auto numbers (with their defaults): PollenHiveSettings.

    /** Autonomous launcher speed in encoder ticks per second: the default in every hive and Odo
     * auto (the INIT menus can change it). The encoder counts 28 ticks per turn, so
     * 1140 ticks/s = 1140 / 28 x 60 = 2443 RPM. v49: 1140 (was 1180), drive team 2026-11-21. */
    public static final int LAUNCHER_TARGET_VELOCITY = 1140;
    /** TeleOp launcher speed: the default in PollenBot TeleOp and Calibration Launcher. v50 kept
     * 1180 when the autos moved to 1140; v51: 1160 (2486 RPM), drive team 2026-11-21. */
    public static final int TELEOP_LAUNCHER_TARGET_VELOCITY = 1160;
    /** The longest the launcher runs alone before feeding starts, in auto, TeleOp and
     * LauncherTest (team-measured). Feeding starts sooner if the launcher is already up to
     * speed: see PollenFeedRule. */
    public static final double LAUNCHER_SPINUP_SECONDS = 3.0;
    /** Agitator power while feeding the launcher, in auto and TeleOp. It replaced a windmill
     * servo that ran at full power while shooting; half power can stall the hopper. */
    public static final double AGITATOR_FEED_POWER = 1.0;
    /** The launcher counts as "up to speed" at this fraction of its target (about 1094 of the
     * 1140 target; originally 1200 of 1250, the StarterBot check). Feeding starts at that speed
     * or after the spin-up time, whichever comes first, so a bad encoder reading can never stop
     * the feed. The rule itself is in PollenFeedRule. */
    public static final double LAUNCHER_READY_FRACTION = 0.96;
    /** TeleOp and LauncherTest: hold L1 to run the launcher BACKWARDS this fast and push a
     * jammed ball back out. 300 RPM = 300 x 28 / 60 = 140 ticks per second. */
    public static final int LAUNCHER_REVERSE_VELOCITY = 140;
    /** TeleOp and LauncherTest: while feeding, this much is added to the intake power from the
     * triggers, so the intake keeps pushing balls toward the launcher. */
    public static final double INTAKE_FEED_BOOST = 0.5;

    /** Full inward intake while autonomous is driving, feeding, or actively collecting. */
    static double autoIntakePower(double forward, double strafe, double turn, boolean handlingPollen) {
        return forward != 0 || strafe != 0 || turn != 0 || handlingPollen ? 1 : 0;
    }

    // Device names in the active PollenBot hardware configuration.
    public static final String[] DRIVE_NAMES = {"frontLeft", "frontRight", "backLeft", "backRight"};
    // Matches the current PollenBot TeleOp and the tested autonomous drivetrain.
    public static final boolean[] DRIVE_REVERSED = {
            true, false, false, true
    };

    public static final double START_DELAY_SECONDS = 0.0; // default; Park can override during INIT
    public static final double TOTAL_TIME_LIMIT = 27.0; // local guard, not a rule claim
    public static final double DEFAULT_PARK_MARGIN = 1.0, PARK_MARGIN_STEP = .5, MAX_PARK_MARGIN = 10;

    // Option A uses timed forward movement; retain the tested defaults.
    // The two manually mirrored start poses use this same forward-only route.
    public static final double PARK_FORWARD_SECONDS = 2;
    public static final double PARK_FORWARD_POWER = 0.20;

    // Positive drive axes = forward/right/clockwise, in route directions (as if facing the hive:
    // PollenFacing turns them around for a robot that starts with its back to the hive).
    // Start pose is set by humans.
    public static PollenSequence.Step[] parkRoute() {
        return parkRoute(PARK_FORWARD_SECONDS, PARK_FORWARD_POWER);
    }

    public static final double PARK_HOLD_SECONDS = 0.5;

    /** Snapshot of the settings selected before START. */
    public static PollenSequence.Step[] parkRoute(double seconds, double power) {
        return new PollenSequence.Step[] {
                PollenSequence.Step.drive("DRIVE TO PARK", seconds, power, 0, 0),
                PollenSequence.Step.drive("PARK / HOLD", PARK_HOLD_SECONDS, 0, 0, 0)
        };
    }

}
