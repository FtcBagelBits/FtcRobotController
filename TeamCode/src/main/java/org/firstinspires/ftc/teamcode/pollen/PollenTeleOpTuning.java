package org.firstinspires.ftc.teamcode.pollen;

/** PollenBot TeleOp INIT menu: launcher speed, spin-up time and drive power. Pure Java, no
 * hardware calls. Defaults come from the TeleOp's own constants and PollenConfig. */
public final class PollenTeleOpTuning {
    public enum Row { LAUNCHER, SPINUP, DRIVE }
    public static final double MIN_LAUNCHER = 500, MAX_LAUNCHER = 2500, LAUNCHER_STEP = 50;
    public static final double MIN_SPINUP = 0, MAX_SPINUP = 6, SPINUP_STEP = .5;
    public static final double MIN_DRIVE = .20, MAX_DRIVE = 1.0, DRIVE_STEP = .05;

    /** Process-local memory between INITs, cleared by a Robot Controller app restart. */
    public static final class Snapshot {
        private final double launcher, spinup, drive;
        private Snapshot(double launcher, double spinup, double drive) {
            this.launcher = launcher;
            this.spinup = spinup;
            this.drive = drive;
        }
    }

    private final double defaultLauncher, defaultSpinup;
    private double launcher, spinup, drive;
    private Row selected = Row.LAUNCHER;
    private boolean locked;
    private final boolean[] previous = new boolean[5];

    /** @param defaultTarget launcher ticks/s; @param defaultSpinupSeconds launcher-only time before feeding. */
    public PollenTeleOpTuning(double defaultTarget, double defaultSpinupSeconds, Snapshot remembered) {
        if (!Double.isFinite(defaultTarget) || defaultTarget < MIN_LAUNCHER || defaultTarget > MAX_LAUNCHER
                || !Double.isFinite(defaultSpinupSeconds) || defaultSpinupSeconds < MIN_SPINUP
                || defaultSpinupSeconds > MAX_SPINUP)
            throw new IllegalArgumentException("Invalid TeleOp launcher defaults");
        defaultLauncher = defaultTarget;
        defaultSpinup = defaultSpinupSeconds;
        launcher = remembered == null ? defaultTarget : remembered.launcher;
        spinup = remembered == null ? defaultSpinupSeconds : remembered.spinup;
        drive = remembered == null ? MAX_DRIVE : remembered.drive;
    }

    /** D-pad UP/DOWN selects, LEFT/RIGHT edits, reset restores defaults. Tap/release; no repeat. */
    public void update(boolean up, boolean down, boolean left, boolean right, boolean reset) {
        if (locked) return;
        boolean[] keys = {up, down, left, right, reset};
        if (reset) {
            if (!previous[4]) { launcher = defaultLauncher; spinup = defaultSpinup; drive = MAX_DRIVE; }
        } else {
            Row[] rows = Row.values();
            if (up && !down && !previous[0]) selected = rows[(selected.ordinal() + rows.length - 1) % rows.length];
            if (down && !up && !previous[1]) selected = rows[(selected.ordinal() + 1) % rows.length];
            int sign = right && !left && !previous[3] ? 1 : left && !right && !previous[2] ? -1 : 0;
            if (sign != 0 && selected == Row.LAUNCHER) launcher = step(launcher, sign * LAUNCHER_STEP, MIN_LAUNCHER, MAX_LAUNCHER);
            if (sign != 0 && selected == Row.SPINUP) spinup = step(spinup, sign * SPINUP_STEP, MIN_SPINUP, MAX_SPINUP);
            if (sign != 0 && selected == Row.DRIVE) drive = step(drive, sign * DRIVE_STEP, MIN_DRIVE, MAX_DRIVE);
        }
        System.arraycopy(keys, 0, previous, 0, keys.length);
    }

    private static double step(double value, double amount, double min, double max) {
        return Math.max(min, Math.min(max, Math.round((value + amount) * 100.0) / 100.0));
    }

    public double launcherVelocity() { return launcher; }
    /** Seconds R1 must be held (launcher on) before the feed boost starts. Speed is not checked. */
    public double spinupSeconds() { return spinup; }
    /** Multiplies all four wheel powers after the normal mecanum normalization. */
    public double driveScale() { return drive; }
    /** True when R1 has been held long enough for the feed boost. */
    public boolean feedReady(double heldSeconds) { return heldSeconds >= spinup; }
    public boolean isDefault() { return launcher == defaultLauncher && spinup == defaultSpinup && drive == MAX_DRIVE; }
    public Row selected() { return selected; }
    public void lock() { locked = true; }
    public boolean locked() { return locked; }
    public Snapshot snapshot() { return new Snapshot(launcher, spinup, drive); }
}
