package org.firstinspires.ftc.teamcode.pollen;

/** PollenBot TeleOp INIT menu: launcher speed and drive power. Pure Java, no hardware calls.
 * Defaults come from the TeleOp's own constants, so an untouched menu changes nothing. */
public final class PollenTeleOpTuning {
    public enum Row { LAUNCHER, DRIVE }
    public static final double MIN_LAUNCHER = 500, MAX_LAUNCHER = 2500, LAUNCHER_STEP = 50;
    public static final double MIN_DRIVE = .20, MAX_DRIVE = 1.0, DRIVE_STEP = .05;

    /** Process-local memory between INITs, cleared by a Robot Controller app restart. */
    public static final class Snapshot {
        private final double launcher, drive;
        private Snapshot(double launcher, double drive) {
            this.launcher = launcher;
            this.drive = drive;
        }
    }

    private final double defaultLauncher, readyFraction;
    private double launcher, drive;
    private Row selected = Row.LAUNCHER;
    private boolean locked;
    private final boolean[] previous = new boolean[5];

    /** @param defaultTarget launcher ticks/s used today; @param defaultMinimum feed-enable speed today. */
    public PollenTeleOpTuning(double defaultTarget, double defaultMinimum, Snapshot remembered) {
        if (!Double.isFinite(defaultTarget) || defaultTarget < MIN_LAUNCHER || defaultTarget > MAX_LAUNCHER
                || !Double.isFinite(defaultMinimum) || defaultMinimum <= 0 || defaultMinimum >= defaultTarget)
            throw new IllegalArgumentException("Invalid TeleOp launcher defaults");
        defaultLauncher = defaultTarget;
        readyFraction = defaultMinimum / defaultTarget; // 1200 / 1250 keeps the same 96% feed gate
        launcher = remembered == null ? defaultTarget : remembered.launcher;
        drive = remembered == null ? MAX_DRIVE : remembered.drive;
    }

    /** D-pad UP/DOWN selects, LEFT/RIGHT edits, reset restores defaults. Tap/release; no repeat. */
    public void update(boolean up, boolean down, boolean left, boolean right, boolean reset) {
        if (locked) return;
        boolean[] keys = {up, down, left, right, reset};
        if (reset) {
            if (!previous[4]) { launcher = defaultLauncher; drive = MAX_DRIVE; }
        } else {
            if ((up && !down && !previous[0]) || (down && !up && !previous[1]))
                selected = selected == Row.LAUNCHER ? Row.DRIVE : Row.LAUNCHER;
            int sign = right && !left && !previous[3] ? 1 : left && !right && !previous[2] ? -1 : 0;
            if (sign != 0 && selected == Row.LAUNCHER) launcher = step(launcher, sign * LAUNCHER_STEP, MIN_LAUNCHER, MAX_LAUNCHER);
            if (sign != 0 && selected == Row.DRIVE) drive = step(drive, sign * DRIVE_STEP, MIN_DRIVE, MAX_DRIVE);
        }
        System.arraycopy(keys, 0, previous, 0, keys.length);
    }

    private static double step(double value, double amount, double min, double max) {
        return Math.max(min, Math.min(max, Math.round((value + amount) * 100.0) / 100.0));
    }

    public double launcherVelocity() { return launcher; }
    /** Feeding is allowed above this speed: the same fraction of target as the original 1200/1250. */
    public double launcherMinVelocity() { return launcher * readyFraction; }
    /** Multiplies all four wheel powers after the normal mecanum normalization. */
    public double driveScale() { return drive; }
    public boolean isDefault() { return launcher == defaultLauncher && drive == MAX_DRIVE; }
    public Row selected() { return selected; }
    public void lock() { locked = true; }
    public boolean locked() { return locked; }
    public Snapshot snapshot() { return new Snapshot(launcher, drive); }
}
