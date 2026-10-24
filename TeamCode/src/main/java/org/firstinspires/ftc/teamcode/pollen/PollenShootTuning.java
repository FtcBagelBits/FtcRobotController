package org.firstinspires.ftc.teamcode.pollen;

import java.util.ArrayList;
import java.util.List;

/** INIT-only selections for the end-wall, forward-facing shooting start. No hardware calls. */
public final class PollenShootTuning {
    public enum Field {
        FLYWHEEL("Launcher ticks/s", 500, 2500, 50, 1250),
        FEED_SECONDS("Feed seconds (0 = skip shot)", 0, 20, .5, 10),
        FEED_POWER("Intake + agitator power", .10, 1, .05, .50),
        START_DELAY("Start delay seconds", 0, 20, .5, 0),
        CALIBRATION("Calibration (0=baseline, 1=mat TRIAL)", 0, 1, 1, 0),
        DRIVE_POWER("Drive power, all legs", .10, .50, .05, .20),
        LEG1_CM("Leg 1 cm, off wall (0 = skip)", 0, 60, 1, 0),
        LEG1_ANGLE("Leg 1 angle deg (+left)", -90, 90, 1, 0),
        LEG2_CM("Leg 2 cm, side clear (0 = skip)", 0, 150, 5, 0),
        LEG2_ANGLE("Leg 2 angle deg (+left)", -90, 90, 1, 90),
        LEG3_CM("Leg 3 cm, to park (0 = skip)", 0, 350, 5, 60.96),
        LEG3_ANGLE("Leg 3 angle deg (+left)", -90, 90, 1, PARK_ANGLE_DEGREES);

        public final String label;
        private final double min, max, step, initial;
        Field(String label, double min, double max, double step, double initial) {
            this.label = label;
            this.min = min;
            this.max = max;
            this.step = step;
            this.initial = initial;
        }
        public boolean angle() { return this == LEG1_ANGLE || this == LEG2_ANGLE || this == LEG3_ANGLE; }
    }

    /** Leg numbers 1-3 run in order after the shot; a zero-cm leg is skipped. */
    public static final int LEGS = 3;
    private static final Field[] LEG_CM = {Field.LEG1_CM, Field.LEG2_CM, Field.LEG3_CM};
    private static final Field[] LEG_ANGLE = {Field.LEG1_ANGLE, Field.LEG2_ANGLE, Field.LEG3_ANGLE};

    public static final double CM_PER_SECOND_PER_POWER = PollenDriveCalibration.BASELINE_COEFFICIENT;
    // Robot-relative translation angle: positive left, zero forward, negative right.
    public static final double PARK_ANGLE_DEGREES = 25.87;
    public static final double SPINUP_TIMEOUT = 4, SPEED_STABLE_SECONDS = .25;
    public static final double READY_FRACTION = .96, HOLD_SECONDS = .5, DEADLINE = 27;

    /** Process-local memory, not saved across a Robot Controller app restart. */
    public static final class Snapshot {
        private final double[] values;
        private Snapshot(double[] values) {
            this.values = values.clone();
        }
    }

    private double[] values = new double[Field.values().length];
    private final boolean supporting;
    private int selected;
    private boolean locked;
    private final boolean[] previous = new boolean[6];
    private String notice = "";

    public PollenShootTuning(Snapshot remembered) {
        this(remembered, false);
    }

    public PollenShootTuning(Snapshot remembered, boolean supporting) {
        this.supporting = supporting;
        reset();
        if (remembered != null) {
            values = remembered.values.clone();
            values[Field.START_DELAY.ordinal()] = supporting ? 15 : 0;
            // A larger remembered route may no longer fit when delay resets.
            if (!validBudget()) reset();
        }
    }

    private void reset() {
        for (Field f : Field.values()) values[f.ordinal()] = f.initial;
        if (supporting) {
            values[Field.START_DELAY.ordinal()] = 15;
            values[Field.FEED_SECONDS.ordinal()] = 7.5;
            values[Field.DRIVE_POWER.ordinal()] = .45;
            values[Field.CALIBRATION.ordinal()] = 1;
            set(1, PollenFieldGeometry.SUPPORT_WALL_CLEARANCE_INCHES * PollenFieldGeometry.INCH_TO_CM, 0);
            set(2, 0, -90);
            set(3, PollenFieldGeometry.supportDistanceCm(), PollenFieldGeometry.supportAngleDegrees());
        }
        notice = supporting ? "UNTESTED flush preset: forward clear of flower, then right to park"
                : "Defaults restored: tested baseline, 2-foot move";
    }

    private void fieldPreset() {
        reset();
        if (!supporting) {
            set(1, PollenFieldGeometry.PRIMARY_WALL_CLEARANCE_CM, 0);
            set(2, PollenFieldGeometry.PRIMARY_SIDE_CLEARANCE_INCHES * PollenFieldGeometry.INCH_TO_CM, 90);
            set(3, PollenFieldGeometry.primaryDistanceCm(), PollenFieldGeometry.primaryAngleDegrees());
            values[Field.DRIVE_POWER.ordinal()] = .25;
            values[Field.CALIBRATION.ordinal()] = 1;
        }
        notice = "UNTESTED field preset loaded; verify pose, full footprint and partner route";
    }

    private void set(int leg, double cm, double angle) {
        values[LEG_CM[leg - 1].ordinal()] = cm;
        values[LEG_ANGLE[leg - 1].ordinal()] = angle;
    }

    /** D-pad selects/edits; triangle resets. Tap/release. */
    public void update(boolean up, boolean down, boolean left, boolean right,
            boolean reset) {
        update(up, down, left, right, reset, false);
    }

    /** Square loads the field preset; triangle restores the mode's own defaults. */
    public void update(boolean up, boolean down, boolean left, boolean right,
            boolean reset, boolean fieldPreset) {
        if (locked) return;
        boolean[] keys = {up, down, left, right, reset, fieldPreset};
        if (reset) {
            if (!previous[4]) reset();
        } else if (fieldPreset) {
            if (!previous[5]) fieldPreset();
        } else {
            if (up && !down && !previous[0]) selected = (selected + Field.values().length - 1) % Field.values().length;
            if (down && !up && !previous[1]) selected = (selected + 1) % Field.values().length;
            if (left && !right && !previous[2]) adjust(-1);
            if (right && !left && !previous[3]) adjust(1);
        }
        System.arraycopy(keys, 0, previous, 0, keys.length);
    }

    private void adjust(int sign) {
        Field f = selected();
        double old = value(f);
        // Preserve fractional defaults; first angle edit reaches the
        // next whole degree in the requested direction, then steps by one degree.
        // This lets the operator select exactly zero without extra controls.
        double next = f.angle()
                ? (sign > 0 ? Math.floor(old) + 1 : Math.ceil(old) - 1)
                : Math.round((old + sign * f.step) * 100.0) / 100.0;
        values[selected] = Math.max(f.min, Math.min(f.max, next));
        if (!validBudget()) {
            values[selected] = old;
            notice = "Change rejected: total must be <26s; support feeding also needs delay >=4s.";
        } else notice = "";
    }

    public Field selected() { return Field.values()[selected]; }
    public double value(Field f) { return values[f.ordinal()]; }
    public String notice() { return notice; }
    public void lock() { locked = true; }
    public boolean locked() { return locked; }
    public Snapshot snapshot() { return new Snapshot(values); }

    private boolean validBudget() {
        return worstCaseSeconds() < DEADLINE - 1
                && (!supporting || value(Field.FEED_SECONDS) == 0 || value(Field.START_DELAY) >= SPINUP_TIMEOUT);
    }

    public boolean supporting() { return supporting; }
    public PollenDriveCalibration.Profile profile() {
        return value(Field.CALIBRATION) == 0 ? PollenDriveCalibration.Profile.TESTED_BASELINE
                : PollenDriveCalibration.Profile.MAT_TRIAL;
    }
    /** Robot-relative translation for leg 1, 2 or 3 at the shared drive power. */
    public PollenDriveCalibration.Move legMove(int leg) {
        if (leg < 1 || leg > LEGS) throw new IllegalArgumentException("Leg must be 1-3");
        return PollenDriveCalibration.move(profile(), value(LEG_CM[leg - 1]),
                value(Field.DRIVE_POWER), value(LEG_ANGLE[leg - 1]));
    }

    /** Leg 3, the final leg to park. */
    public PollenDriveCalibration.Move move() {
        return legMove(3);
    }

    public double driveSeconds() {
        return move().seconds;
    }

    public double legCm(int leg) { return value(LEG_CM[leg - 1]); }
    public double legAngle(int leg) { return value(LEG_ANGLE[leg - 1]); }

    public String legName(int leg) {
        String[] names = supporting
                ? new String[] {"LEG 1 CLEAR FLOWER", "LEG 2 (OPTIONAL)", "LEG 3 TO PARK"}
                : new String[] {"LEG 1 OFF WALL", "LEG 2 CLEAR HIVE", "LEG 3 TO PARK"};
        return names[leg - 1];
    }

    public double worstCaseSeconds() {
        double drive = 0;
        for (int leg = 1; leg <= LEGS; leg++) drive += legMove(leg).seconds;
        return value(Field.START_DELAY) + drive + HOLD_SECONDS
                + (value(Field.FEED_SECONDS) > 0 ? (supporting ? 0 : SPINUP_TIMEOUT) + value(Field.FEED_SECONDS) : 0);
    }

    // Both alliance starts are related by a 180-degree rotation. Humans position
    // the robot facing its hive; the same robot-relative vector works on either side.
    public double forward() {
        return move().forward;
    }
    public double strafe() {
        return move().strafe;
    }

    public PollenSequence sequence() {
        return sequence(true);
    }

    /** Shooter-init fallback follows THIS starting pose's departure, never Park A's route. */
    public PollenSequence sequence(boolean shooterAvailable) {
        List<PollenSequence.Step> route = new ArrayList<>();
        if (shooterAvailable && value(Field.FEED_SECONDS) > 0) route.add(PollenSequence.Step.continuousShoot(value(Field.FEED_SECONDS)));
        if (supporting && !shooterAvailable && value(Field.FEED_SECONDS) > 0)
            route.add(PollenSequence.Step.drive("NO SHOOTER / WAIT FOR DEPARTURE", value(Field.FEED_SECONDS), 0, 0, 0));
        for (int leg = 1; leg <= LEGS; leg++) {
            if (legCm(leg) <= 0) continue;
            PollenDriveCalibration.Move m = legMove(leg);
            route.add(PollenSequence.Step.drive(legName(leg), m.seconds, m.forward, m.strafe, 0));
        }
        route.add(PollenSequence.Step.drive("PARK / HOLD", HOLD_SECONDS, 0, 0, 0));
        return new PollenSequence(route.toArray(new PollenSequence.Step[0]), value(Field.FLYWHEEL),
                value(Field.FLYWHEEL) * READY_FRACTION, SPINUP_TIMEOUT, SPEED_STABLE_SECONDS,
                DEADLINE, value(Field.START_DELAY), supporting && shooterAvailable && value(Field.FEED_SECONDS) > 0);
    }
}
