package org.firstinspires.ftc.teamcode.pollen;

/** PollenBot TeleOp INIT menu: launcher speed, spin-up time, agitator feed power, drive power and optional drainer settings. Pure Java, no
 * hardware calls. Defaults come from the TeleOp's own constants and PollenConfig. */
public final class PollenTeleOpTuning {
    public enum Row { LAUNCHER, SPINUP, AGITATOR, DRIVE, DRAINER_UP, DRAINER_DOWN, DRAINER_STEP }
    public static final double MIN_LAUNCHER = 500, MAX_LAUNCHER = 2500, LAUNCHER_STEP = 20;
    public static final double MIN_SPINUP = 0, MAX_SPINUP = 6, SPINUP_STEP = .5;
    public static final double MIN_AGITATOR = .10, MAX_AGITATOR = 1.0, AGITATOR_STEP = .05;
    public static final double MIN_DRIVE = .20, MAX_DRIVE = 1.0, DRIVE_STEP = .05;

    public static final double DEFAULT_DRAINER_UP = .45, DEFAULT_DRAINER_DOWN = .02, DEFAULT_DRAINER_STEP = .01;
    public static final double DRAINER_POSITION_INCREMENT = .01, DRAINER_STEP_INCREMENT = .002;
    public static final double MIN_DRAINER_STEP = .002, MAX_DRAINER_STEP = 1;

    /** Process-local memory between INITs, cleared by a Robot Controller app restart. */
    public static final class Snapshot {
        private final double launcher, spinup, agitator, drive, drainerUp, drainerDown, drainerStep;
        private Snapshot(double launcher, double spinup, double agitator, double drive,
                double drainerUp, double drainerDown, double drainerStep) {
            this.launcher = launcher;
            this.spinup = spinup;
            this.agitator = agitator;
            this.drive = drive;
            this.drainerUp = drainerUp;
            this.drainerDown = drainerDown;
            this.drainerStep = drainerStep;
        }
    }

    private final double defaultLauncher, defaultSpinup, defaultAgitator;
    private double launcher, spinup, agitator, drive;
    private double drainerUp, drainerDown, drainerStep;
    private final boolean drainerEnabled;
    private Row selected = Row.LAUNCHER;
    private int firstRow;
    private boolean locked;
    private final boolean[] previous = new boolean[5];

    /** @param defaultTarget launcher ticks/s; @param defaultSpinupSeconds launcher-only time before
     * feeding; @param defaultAgitatorPower agitator power while feeding. */
    public PollenTeleOpTuning(double defaultTarget, double defaultSpinupSeconds, double defaultAgitatorPower,
            Snapshot remembered) {
        this(defaultTarget, defaultSpinupSeconds, defaultAgitatorPower, remembered, false);
    }

    /** Enable the extra drainer rows only in PollenBot, which has the servo. */
    public PollenTeleOpTuning(double defaultTarget, double defaultSpinupSeconds, double defaultAgitatorPower,
            Snapshot remembered, boolean drainerEnabled) {
        this.drainerEnabled = drainerEnabled;
        if (!Double.isFinite(defaultTarget) || defaultTarget < MIN_LAUNCHER || defaultTarget > MAX_LAUNCHER
                || !Double.isFinite(defaultSpinupSeconds) || defaultSpinupSeconds < MIN_SPINUP
                || defaultSpinupSeconds > MAX_SPINUP || !Double.isFinite(defaultAgitatorPower)
                || defaultAgitatorPower < MIN_AGITATOR || defaultAgitatorPower > MAX_AGITATOR)
            throw new IllegalArgumentException("Invalid TeleOp launcher defaults");
        defaultLauncher = defaultTarget;
        defaultSpinup = defaultSpinupSeconds;
        defaultAgitator = defaultAgitatorPower;
        launcher = remembered == null ? defaultTarget : remembered.launcher;
        spinup = remembered == null ? defaultSpinupSeconds : remembered.spinup;
        agitator = remembered == null ? defaultAgitatorPower : remembered.agitator;
        drive = remembered == null ? MAX_DRIVE : remembered.drive;
        drainerUp = remembered == null ? DEFAULT_DRAINER_UP : remembered.drainerUp;
        drainerDown = remembered == null ? DEFAULT_DRAINER_DOWN : remembered.drainerDown;
        drainerStep = remembered == null ? DEFAULT_DRAINER_STEP : remembered.drainerStep;
    }

    /** Servo-only menu for PollenDrainerTest; shares the TeleOp defaults and edit rules. */
    public static PollenTeleOpTuning drainerOnly(Snapshot remembered) {
        PollenTeleOpTuning tuning = new PollenTeleOpTuning(PollenConfig.LAUNCHER_TARGET_VELOCITY,
                PollenConfig.LAUNCHER_SPINUP_SECONDS, PollenConfig.AGITATOR_FEED_POWER, remembered, true);
        tuning.firstRow = Row.DRAINER_UP.ordinal();
        tuning.selected = Row.DRAINER_UP;
        return tuning;
    }

    /** D-pad UP/DOWN selects, LEFT/RIGHT edits, reset restores defaults. Tap/release; no repeat. */
    public void update(boolean up, boolean down, boolean left, boolean right, boolean reset) {
        if (locked) return;
        boolean[] keys = {up, down, left, right, reset};
        if (reset) {
            if (!previous[4]) { launcher = defaultLauncher; spinup = defaultSpinup; agitator = defaultAgitator; drive = MAX_DRIVE;
                drainerUp = DEFAULT_DRAINER_UP; drainerDown = DEFAULT_DRAINER_DOWN; drainerStep = DEFAULT_DRAINER_STEP;
            }
        } else {
            Row[] rows = Row.values();
            int rowCount = (drainerEnabled ? rows.length : Row.DRIVE.ordinal() + 1) - firstRow;
            if (up && !down && !previous[0]) selected = rows[firstRow + (selected.ordinal() - firstRow + rowCount - 1) % rowCount];
            if (down && !up && !previous[1]) selected = rows[firstRow + (selected.ordinal() - firstRow + 1) % rowCount];
            int sign = right && !left && !previous[3] ? 1 : left && !right && !previous[2] ? -1 : 0;
            if (sign != 0 && selected == Row.LAUNCHER) launcher = step(launcher, sign * LAUNCHER_STEP, MIN_LAUNCHER, MAX_LAUNCHER);
            if (sign != 0 && selected == Row.SPINUP) spinup = step(spinup, sign * SPINUP_STEP, MIN_SPINUP, MAX_SPINUP);
            if (sign != 0 && selected == Row.AGITATOR) agitator = step(agitator, sign * AGITATOR_STEP, MIN_AGITATOR, MAX_AGITATOR);
            if (sign != 0 && selected == Row.DRIVE) drive = step(drive, sign * DRIVE_STEP, MIN_DRIVE, MAX_DRIVE);
            if (sign != 0 && selected == Row.DRAINER_UP) drainerUp = step(drainerUp, sign * DRAINER_POSITION_INCREMENT, 0, 1);
            if (sign != 0 && selected == Row.DRAINER_DOWN) drainerDown = step(drainerDown, sign * DRAINER_POSITION_INCREMENT, 0, 1);
            if (sign != 0 && selected == Row.DRAINER_STEP) drainerStep = step(drainerStep, sign * DRAINER_STEP_INCREMENT, MIN_DRAINER_STEP, MAX_DRAINER_STEP);
        }
        System.arraycopy(keys, 0, previous, 0, keys.length);
    }

    private static double step(double value, double amount, double min, double max) {
        return Math.max(min, Math.min(max, Math.round((value + amount) * 1000.0) / 1000.0));
    }

    public double drainerUpPosition() { return drainerUp; }
    public double drainerDownPosition() { return drainerDown; }
    public double drainerStep() { return drainerStep; }
    public double launcherVelocity() { return launcher; }
    /** Longest wait, in seconds of R1, before feeding starts (sooner if up to speed). */
    public double spinupSeconds() { return spinup; }
    /** Agitator power while feeding (R1 held past the spin-up). */
    public double agitatorPower() { return agitator; }
    /** Multiplies all four wheel powers after the normal mecanum normalization. */
    public double driveScale() { return drive; }
    /** "Up to speed" means 96% of the target: about 1133 ticks/s for the default 1180. */
    public double launcherReadyVelocity() { return launcher * PollenConfig.LAUNCHER_READY_FRACTION; }

    /** Feeding rule 1: the launcher's encoder says it is up to speed (PollenFeedRule). */
    public boolean launcherIsUpToSpeed(double measuredVelocity) {
        return PollenFeedRule.upToSpeed(measuredVelocity, launcher);
    }

    /** Feeding rule 2 (the backup): R1 has been held for the whole spin-up time, so feed even
     * if the encoder reading is wrong. This is the rule that fixed the October 24 stall. */
    public boolean spinUpTimeIsOver(double heldSeconds) { return PollenFeedRule.spinUpTimeIsOver(heldSeconds, spinup); }
    public boolean isDefault() {
        return launcher == defaultLauncher && spinup == defaultSpinup && agitator == defaultAgitator && drive == MAX_DRIVE
                && (!drainerEnabled || (drainerUp == DEFAULT_DRAINER_UP && drainerDown == DEFAULT_DRAINER_DOWN
                && drainerStep == DEFAULT_DRAINER_STEP));
    }
    public Row selected() { return selected; }
    public void lock() { locked = true; }
    public boolean locked() { return locked; }
    public Snapshot snapshot() { return new Snapshot(launcher, spinup, agitator, drive, drainerUp, drainerDown, drainerStep); }
}
