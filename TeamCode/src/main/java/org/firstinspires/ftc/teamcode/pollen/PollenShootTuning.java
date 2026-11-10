package org.firstinspires.ftc.teamcode.pollen;

import java.util.ArrayList;
import java.util.List;

/** INIT-only selections for the end-wall, forward-facing shooting start. No hardware calls.
 *
 * ODOMETRY MODE (the "Odo" OpModes): the same menu, but each leg's cm and angle are a real
 * distance and direction, driven to with odometry (PollenOdometryDrive) instead of being turned
 * into seconds. The CALIBRATION row becomes "wall square first": at the very start, back into
 * the end wall so the robot sits flat against it, then zero the odometry there. Doing it first
 * squares the shot as well as the route, and it runs inside the start delay when there is one
 * (C's 12 s), so C still feeds at 15 s. There is no end hold: a leg only ends once the robot is
 * stopped inside the tolerance. Leg 3 defaults to the CAD plan (PollenFieldGeometry), not the
 * field-tuned timed values: those were shortened (17-21%), most likely to make up for timed driving
 * overshooting its model, so as true distances they would stop B about 19 in and C about 8 in short of the planned spot.
 *
 * ANGLED MODE (the experimental "Angled Primary Shooter", odometry only): the same three legs, in
 * a different order. Legs 1 and 2 run FIRST and take the robot to the shooting spot, off the wall
 * and to the side of the hive. There it turns in place to the AIM_ANGLE row, shoots, and turns
 * back. Leg 3 then goes to the same parking spot as the normal route. */
public final class PollenShootTuning {
    public enum Field {
        FLYWHEEL("Launcher ticks/s", 500, 2500, 20, PollenConfig.LAUNCHER_TARGET_VELOCITY),
        SPINUP("Longest spin-up before feed (s)", 0, 6, .5, PollenConfig.LAUNCHER_SPINUP_SECONDS),
        FEED_SECONDS("Feed seconds (0 = skip shot)", 0, 20, .5, 10),
        FEED_POWER("Intake power while feeding", .10, 1, .05, .50),
        AGITATOR_POWER("Agitator power while feeding", .10, 1, .05, PollenConfig.AGITATOR_FEED_POWER),
        START_DELAY("Start delay seconds", 0, 20, .5, 0),
        CALIBRATION("Calibration (0=baseline, 1=mat TRIAL)", 0, 1, 1, 0),
        DRIVE_POWER("Drive power, all legs", .10, .50, .05, .20),
        LEG1_CM("Leg 1 cm, off wall (0 = skip)", 0, 60, 1, 0),
        LEG1_ANGLE("Leg 1 angle deg (+left)", -90, 90, 1, 0),
        LEG2_CM("Leg 2 cm, side clear (0 = skip)", 0, 150, 5, 0),
        LEG2_ANGLE("Leg 2 angle deg (+left)", -90, 90, 1, 90),
        LEG3_CM("Leg 3 cm, to park (0 = skip)", 0, 350, 5, 0),
        LEG3_ANGLE("Leg 3 angle deg (+left)", -90, 90, 1, 0),
        /** Angled mode only (see rows()): the heading the robot turns to for the shot. */
        AIM_ANGLE("Aim angle deg for the shot (+left)", -60, 60, 1, 0);

        public final String label;
        private final double min, max, step, initial;
        Field(String label, double min, double max, double step, double initial) {
            this.label = label;
            this.min = min;
            this.max = max;
            this.step = step;
            this.initial = initial;
        }
        public boolean angle() { return this == LEG1_ANGLE || this == LEG2_ANGLE || this == LEG3_ANGLE || this == AIM_ANGLE; }
    }

    /** Leg numbers 1-3 run in order after the shot; a zero-cm leg is skipped. */
    public static final int LEGS = 3;
    private static final Field[] LEG_CM = {Field.LEG1_CM, Field.LEG2_CM, Field.LEG3_CM};
    private static final Field[] LEG_ANGLE = {Field.LEG1_ANGLE, Field.LEG2_ANGLE, Field.LEG3_ANGLE};

    public static final double CM_PER_SECOND_PER_POWER = PollenDriveCalibration.BASELINE_COEFFICIENT;
    public static final double HOLD_SECONDS = .5, DEADLINE = 27;
    /** C waits this long, then spins up for 3 s, so its feed starts at 15 s as agreed with partners. */
    public static final double SUPPORT_START_DELAY = 12;
    // Field-tuned last-leg defaults set by the team on 2026-10-24. They replace the CAD plan's
    // computed values (PollenFieldGeometry), which remain the nominal geometry reference.
    public static final double B_FIELD_LEG3_CM = 172.47, B_FIELD_LEG3_DEGREES = 17;
    public static final double C_LEG3_CM = 97.96, C_LEG3_DEGREES = -81.85;
    /** Angled mode: the least gap allowed between the end wall and the robot's nearest corner
     * while it turns at the shooting spot. An edit that leaves less is rejected. */
    public static final double MIN_TURN_WALL_GAP_INCHES = 0.5;

    /** Which route the rows hold, shown as the first line of the INIT screen and in the log. */
    public enum Route {
        FULL_PARK("FULL PARK: shoot, then 3 legs to the parking slot"),
        ANGLED_PARK("ANGLED SHOT: 2 legs to the shooting spot, turn, shoot, turn back, park"),
        SUPPORT_PARK("SUPPORT PARK: wait, shoot, then park in the outer slot"),
        EDITED("EDITED LEGS: check every leg row before START");
        public final String label;
        Route(String label) { this.label = label; }
    }

    /** Process-local memory, not saved across a Robot Controller app restart. */
    public static final class Snapshot {
        private final double[] values;
        private final Route route;
        private Snapshot(double[] values, Route route) {
            this.values = values.clone();
            this.route = route;
        }
    }

    private double[] values = new double[Field.values().length];
    private final boolean supporting, odometry, angled;
    private final Field[] rows;
    private int selected;
    private boolean locked;
    private final boolean[] previous = new boolean[5];
    private String notice = "";
    private Route route;

    public PollenShootTuning(Snapshot remembered) {
        this(remembered, false);
    }

    public PollenShootTuning(Snapshot remembered, boolean supporting) {
        this(remembered, supporting, false);
    }

    public PollenShootTuning(Snapshot remembered, boolean supporting, boolean odometry) {
        this(remembered, supporting, odometry, false);
    }

    /** @param angled the experimental Angled Primary Shooter: odometry, and not the supporting shooter. */
    public PollenShootTuning(Snapshot remembered, boolean supporting, boolean odometry, boolean angled) {
        if (angled && (supporting || !odometry))
            throw new IllegalArgumentException("The angled shot needs odometry and is for the primary shooter only");
        this.supporting = supporting;
        this.odometry = odometry;
        this.angled = angled;
        // The AIM_ANGLE row only exists in angled mode. It is the last Field, so in every other
        // mode the rows are simply all the Fields before it.
        rows = angled ? Field.values() : java.util.Arrays.copyOf(Field.values(), Field.AIM_ANGLE.ordinal());
        reset();
        if (remembered != null) {
            values = remembered.values.clone();
            route = remembered.route;
            values[Field.START_DELAY.ordinal()] = supporting ? SUPPORT_START_DELAY : 0;
            // Keep the remembered (possibly calibrated) route even if the restored delay makes it
            // too long: the robot will not start until the operator fixes the budget or presses
            // Triangle. Never swap in different distances, angles or power without saying so.
            notice = budgetOk() ? "Remembered settings restored; start delay reset to its default"
                    : overBudgetNotice();
        }
    }

    /** Triangle, and every INIT without remembered settings. B: shoot, then the 3-leg route to
     * the parking slot. C: its flush route to the outer slot. */
    private void reset() {
        initialValues();
        route = supporting ? Route.SUPPORT_PARK : angled ? Route.ANGLED_PARK : Route.FULL_PARK;
        notice = angled ? "Defaults: 2 legs to the shooting spot, turn, shoot, turn back, park (EXPERIMENTAL)"
                : supporting
                ? (odometry ? "Defaults: forward clear of flower, then right to the planned park spot"
                            : "Flush preset: forward clear of flower, then right to park (leg 3 field-tuned)")
                : (odometry ? "Defaults: shoot, then 3 legs to the parking slot (planned spot)"
                            : "Defaults: shoot, then 3 legs to the parking slot (leg 3 field-tuned)");
    }

    /** Every row's default: B's full route, or C's support route. */
    private void initialValues() {
        for (Field f : Field.values()) values[f.ordinal()] = f.initial;
        if (!supporting) fullRoute();
        if (supporting) {
            values[Field.START_DELAY.ordinal()] = SUPPORT_START_DELAY;
            values[Field.FEED_SECONDS.ordinal()] = 7.5;
            values[Field.DRIVE_POWER.ordinal()] = .45;
            values[Field.CALIBRATION.ordinal()] = 1;
            set(1, PollenFieldGeometry.SUPPORT_WALL_CLEARANCE_INCHES * PollenFieldGeometry.INCH_TO_CM, 0);
            set(2, 0, -90);
            set(3, C_LEG3_CM, C_LEG3_DEGREES);
        }
        if (odometry && supporting) {
            values[Field.CALIBRATION.ordinal()] = 1; // wall square ON
            values[Field.DRIVE_POWER.ordinal()] = PollenOdometryDrive.DEFAULT_LEG_POWER;
            set(3, PollenFieldGeometry.supportDistanceCm(), PollenFieldGeometry.supportAngleDegrees());
        }
        if (angled) angledRoute();
    }

    /** Angled mode: legs 1 and 2 go to the shooting spot (off the wall, then to the left), the
     * aim angle points the robot at the hive opening from there, and leg 3 goes to the same
     * parking spot as fullRoute(). The spot is the team's ASSUMED one; it is not field-tested. */
    private void angledRoute() {
        set(1, PollenFieldGeometry.ANGLED_OFF_WALL_INCHES * PollenFieldGeometry.INCH_TO_CM, 0);
        set(2, PollenFieldGeometry.ANGLED_SIDE_INCHES * PollenFieldGeometry.INCH_TO_CM, 90);
        set(3, PollenFieldGeometry.angledDistanceCm(), PollenFieldGeometry.angledAngleDegrees());
        values[Field.AIM_ANGLE.ordinal()] = PollenFieldGeometry.angledAimDegrees();
    }

    /** B's full route: off the wall, left past the frame, then to the inner parking slot. */
    private void fullRoute() {
        set(1, PollenFieldGeometry.PRIMARY_WALL_CLEARANCE_CM, 0);
        set(2, PollenFieldGeometry.PRIMARY_SIDE_CLEARANCE_INCHES * PollenFieldGeometry.INCH_TO_CM, 90);
        set(3, B_FIELD_LEG3_CM, B_FIELD_LEG3_DEGREES);
        values[Field.DRIVE_POWER.ordinal()] = .25;
        values[Field.CALIBRATION.ordinal()] = 1;
        if (odometry) {
            set(3, PollenFieldGeometry.primaryDistanceCm(), PollenFieldGeometry.primaryAngleDegrees());
            values[Field.DRIVE_POWER.ordinal()] = PollenOdometryDrive.DEFAULT_LEG_POWER;
        }
    }

    private void set(int leg, double cm, double angle) {
        values[LEG_CM[leg - 1].ordinal()] = cm;
        values[LEG_ANGLE[leg - 1].ordinal()] = angle;
    }

    /** D-pad selects/edits; Triangle (reset) restores the defaults. Tap/release. Circle and
     * Square are not read: Square used to load B's full route (now the default), and Circle
     * loaded a short "leave the wall" move that was removed in pollen-odo-v6. */
    public void update(boolean up, boolean down, boolean left, boolean right, boolean reset) {
        if (locked) return;
        boolean[] keys = {up, down, left, right, reset};
        if (reset) {
            if (!previous[4]) reset();
        } else {
            if (up && !down && !previous[0]) selected = (selected + rows.length - 1) % rows.length;
            if (down && !up && !previous[1]) selected = (selected + 1) % rows.length;
            if (left && !right && !previous[2]) adjust(-1);
            if (right && !left && !previous[3]) adjust(1);
        }
        System.arraycopy(keys, 0, previous, 0, keys.length);
    }

    private void adjust(int sign) {
        Field f = selected();
        double old = value(f), oldWorst = worstCaseSeconds();
        boolean wasOk = budgetOk();
        // Preserve fractional defaults; first angle edit reaches the
        // next whole degree in the requested direction, then steps by one degree.
        // This lets the operator select exactly zero without extra controls.
        double next = f.angle()
                ? (sign > 0 ? Math.floor(old) + 1 : Math.ceil(old) - 1)
                : Math.round((old + sign * f.step) * 100.0) / 100.0;
        // Odometry legs may run faster than timed ones (timed calibration only covers 0.10-0.50).
        double max = odometry && f == Field.DRIVE_POWER ? PollenOdometryDrive.MAX_LEG_POWER : f.max;
        boolean hadRoomToTurn = roomToTurn();
        values[f.ordinal()] = Math.max(f.min, Math.min(max, next));
        // Over budget (e.g. remembered settings after the delay reset), accept any edit that
        // does not lengthen the route, so the operator can work back under the limit.
        boolean accepted = budgetOk() || (!wasOk && worstCaseSeconds() <= oldWorst + 1e-9);
        if (accepted && hadRoomToTurn && !roomToTurn()) {
            // Angled mode: never accept an edit that would swing a corner into the end wall.
            values[f.ordinal()] = old;
            notice = "Change rejected: the robot would hit the wall turning there. Raise Leg 1 first.";
        } else if (!accepted) {
            values[f.ordinal()] = old;
            notice = wasOk ? "Change rejected: total must be <26s."
                    : overBudgetNotice() + " That change made it longer.";
        } else notice = budgetOk() ? "" : overBudgetNotice();
        if (accepted && value(f) != old && isLegRow(f)) route = Route.EDITED;
    }

    private static boolean isLegRow(Field f) {
        for (int i = 0; i < LEGS; i++) if (f == LEG_CM[i] || f == LEG_ANGLE[i]) return true;
        return f == Field.AIM_ANGLE;
    }

    /** The menu rows of this mode, top to bottom. */
    public Field[] rows() { return rows.clone(); }
    /** The experimental Angled Primary Shooter. */
    public boolean angled() { return angled; }
    /** Angled mode: the heading the robot turns to for the shot, in degrees (+left). 0 = no turn. */
    public double aimDegrees() { return angled ? value(Field.AIM_ANGLE) : 0; }
    /** True if this route turns before and after the shot. */
    private boolean turnsToAim() { return angled && value(Field.FEED_SECONDS) > 0 && aimDegrees() != 0; }
    /** Angled mode: expected seconds for ONE of the two turns (0 if the route does not turn). */
    public double aimTurnSeconds() {
        return turnsToAim() ? PollenOdometryDrive.turnPlannedSeconds(aimDegrees(), PollenOdometryDrive.DEFAULT_TURN_POWER) : 0;
    }

    /** Angled mode: the gap, in inches, between the end wall and the robot's nearest corner at
     * the worst moment of the turn at the shooting spot. The zero is taken with the robot's back
     * against the wall, so its center starts ROBOT_HALF_INCHES from the wall and legs 1 and 2
     * move it forward from there. A square robot turned by an angle reaches
     * half x (cos + sin) toward the wall, and the most at 45 degrees. */
    public double turnWallGapInches() {
        double half = PollenFieldGeometry.ROBOT_HALF_INCHES;
        double worstAngle = Math.toRadians(Math.min(45, Math.abs(aimDegrees())));
        double centerFromWall = half + legEndInches(2)[0];
        return centerFromWall - half * (Math.cos(worstAngle) + Math.sin(worstAngle));
    }

    /** False only when an angled route would swing a corner into (or too near) the end wall. */
    public boolean roomToTurn() {
        return !turnsToAim() || turnWallGapInches() >= MIN_TURN_WALL_GAP_INCHES;
    }

    public Route route() { return route; }

    /** One line with every setting, for the robot log at START. */
    public String settingsLine() {
        StringBuilder b = new StringBuilder("route=").append(route);
        for (Field f : rows) b.append(' ').append(f.name()).append('=').append(value(f));
        return b.toString();
    }

    public Field selected() { return rows[selected]; }
    public boolean odometry() { return odometry; }
    /** Row label as shown on the Driver Hub; one row means something else in odometry mode. */
    public String label(Field f) {
        if (angled && f == Field.LEG2_CM) return "Leg 2 cm, to the shooting spot (0 = skip)";
        return odometry && f == Field.CALIBRATION ? "Wall square first (0=off, 1=on)" : f.label;
    }
    /** Odometry mode: at the start, back into the end wall and zero the odometry there. */
    public boolean wallSquare() { return odometry && value(Field.CALIBRATION) == 1; }

    /** Odometry mode: where leg 1, 2 or 3 ends, in inches from the zero point {forward, left}.
     * Legs add up, so a skipped (0 cm) leg ends where the previous one did. */
    public double[] legEndInches(int leg) {
        if (leg < 1 || leg > LEGS) throw new IllegalArgumentException("Leg must be 1-3");
        double[] end = {0, 0};
        for (int i = 1; i <= leg; i++) end = PollenOdometryDrive.legEnd(end[0], end[1], legCm(i), legAngle(i));
        return end;
    }

    /** Odometry mode: expected seconds for one leg at the shared drive power (0 if skipped). */
    public double odometryLegSeconds(int leg) {
        return legCm(leg) <= 0 ? 0 : PollenOdometryDrive.plannedSeconds(legCm(leg), value(Field.DRIVE_POWER));
    }
    public double value(Field f) { return values[f.ordinal()]; }
    public String notice() { return notice; }
    public void lock() { locked = true; }
    public boolean locked() { return locked; }
    public Snapshot snapshot() { return new Snapshot(values, route); }

    /** True when these settings fit the time budget; START refuses otherwise. */
    public boolean budgetOk() {
        return worstCaseSeconds() < DEADLINE - 1;
    }

    /** Seconds from START until feeding begins AT THE LATEST (delay + the whole spin-up time),
     * assuming a normal loop. The primary shooter feeds sooner if its launcher is up to speed
     * sooner; the Supporting Shooter always feeds at exactly this time. */
    public double feedStartSeconds() {
        return shotStartSeconds() + value(Field.SPINUP);
    }

    /** True if the shot may start feeding before the whole spin-up time is over. The Supporting
     * Shooter may not: its feed time is agreed with the partner, who shoots first. */
    public boolean feedsWhenUpToSpeed() { return !supporting; }

    /** Seconds from START until the launcher turns on. In odometry mode the squaring and zero
     * come first, inside the start delay when it is long enough. In angled mode legs 1 and 2
     * and the turn to the aim angle come before the shot as well. */
    public double shotStartSeconds() {
        double start = odometry ? Math.max(value(Field.START_DELAY), odometryPrepSeconds()) : value(Field.START_DELAY);
        if (angled) start += odometryLegSeconds(1) + odometryLegSeconds(2) + aimTurnSeconds();
        return start;
    }

    /** Odometry mode: wall square (if on) plus the zero, done before anything else. */
    private double odometryPrepSeconds() {
        return (wallSquare() ? PollenOdometryDrive.SQUARE_SECONDS : 0) + PollenOdometryDrive.ZERO_SETTLE_SECONDS;
    }

    private String overBudgetNotice() {
        return String.format(java.util.Locale.US,
                "OVER TIME BUDGET: %.2f s planned, must be < %.0f s. Lower delay or feed, raise power, "
                        + "or shorten legs; Triangle = defaults. Will not start.", worstCaseSeconds(), DEADLINE - 1);
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
                : angled ? new String[] {"LEG 1 OFF WALL", "LEG 2 TO SHOOTING SPOT", "LEG 3 TO PARK"}
                : new String[] {"LEG 1 OFF WALL", "LEG 2 CLEAR HIVE", "LEG 3 TO PARK"};
        return names[leg - 1];
    }

    /** Timed mode: every leg's fixed time. Odometry mode: each leg's expected time, plus the
     * wall square and zero; the hard stop at DEADLINE still applies if the robot is slower. */
    public double worstCaseSeconds() {
        double drive = 0;
        if (angled) {
            // shotStartSeconds() already counts legs 1 and 2 and the turn to the aim angle.
            return shotStartSeconds() + aimTurnSeconds() + odometryLegSeconds(3)
                    + (value(Field.FEED_SECONDS) > 0 ? value(Field.SPINUP) + value(Field.FEED_SECONDS) : 0);
        }
        if (odometry) {
            for (int leg = 1; leg <= LEGS; leg++) drive += odometryLegSeconds(leg);
            return shotStartSeconds() + drive
                    + (value(Field.FEED_SECONDS) > 0 ? value(Field.SPINUP) + value(Field.FEED_SECONDS) : 0);
        }
        for (int leg = 1; leg <= LEGS; leg++) drive += legMove(leg).seconds;
        return value(Field.START_DELAY) + drive + HOLD_SECONDS
                + (value(Field.FEED_SECONDS) > 0 ? value(Field.SPINUP) + value(Field.FEED_SECONDS) : 0);
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

    /** Builds the route these settings describe. Without a working shooter the same departure
     * is driven with the shot (and, in angled mode, the two turns) left out. */
    public PollenSequence sequence(boolean shooterAvailable) {
        List<PollenSequence.Step> route = new ArrayList<>();
        boolean shoots = shooterAvailable && value(Field.FEED_SECONDS) > 0;
        if (odometry) {
            if (wallSquare()) route.add(PollenSequence.Step.drive("SQUARE TO WALL",
                    PollenOdometryDrive.SQUARE_SECONDS, -PollenOdometryDrive.SQUARE_POWER, 0, 0));
            // Every leg is measured from here, so the legs end at the same field spots even if
            // the robot gets nudged while it shoots.
            route.add(PollenSequence.Step.zeroOdometry(PollenOdometryDrive.ZERO_SETTLE_SECONDS));
        }
        // Angled mode: drive to the shooting spot BEFORE the shot (the other modes shoot first).
        if (angled) {
            addLeg(route, 1);
            addLeg(route, 2);
        }
        boolean turns = shoots && turnsToAim();
        if (turns) route.add(aimTurn("TURN TO AIM", aimDegrees()));
        // The shot. The Supporting Shooter waits the whole spin-up time, so it feeds at the time
        // agreed with the partner; the primary shooter feeds as soon as its launcher is ready.
        if (shoots) route.add(feedsWhenUpToSpeed() ? PollenSequence.Step.continuousShoot(value(Field.FEED_SECONDS))
                : PollenSequence.Step.shootAtSetTime(value(Field.FEED_SECONDS)));
        if (turns) route.add(aimTurn("TURN BACK", 0));
        if (supporting && !shooterAvailable && value(Field.FEED_SECONDS) > 0)
            // Hold still through the spin-up and feed slot so C still departs after the partner.
            route.add(PollenSequence.Step.drive("NO SHOOTER / WAIT FOR DEPARTURE",
                    value(Field.SPINUP) + value(Field.FEED_SECONDS), 0, 0, 0));
        for (int leg = angled ? 3 : 1; leg <= LEGS; leg++) addLeg(route, leg);
        if (!odometry) route.add(PollenSequence.Step.drive("PARK / HOLD", HOLD_SECONDS, 0, 0, 0));
        // Odometry: the prep steps run inside the start delay, so the shot still starts on time.
        double delay = odometry ? Math.max(0, value(Field.START_DELAY) - odometryPrepSeconds()) : value(Field.START_DELAY);
        return new PollenSequence(route.toArray(new PollenSequence.Step[0]), value(Field.FLYWHEEL),
                value(Field.SPINUP), DEADLINE, delay);
    }

    /** Adds leg 1, 2 or 3 to the route: an odometry move, or a timed drive in the timed modes.
     * A leg of 0 cm is skipped. */
    private void addLeg(List<PollenSequence.Step> route, int leg) {
        if (legCm(leg) <= 0) return;
        if (odometry) {
            double[] end = legEndInches(leg);
            double power = value(Field.DRIVE_POWER);
            route.add(PollenSequence.Step.moveTo(legName(leg), end[0], end[1], power,
                    PollenOdometryDrive.plannedSeconds(legCm(leg), power),
                    PollenOdometryDrive.timeoutSeconds(legCm(leg), power)));
            return;
        }
        PollenDriveCalibration.Move m = legMove(leg);
        route.add(PollenSequence.Step.drive(legName(leg), m.seconds, m.forward, m.strafe, 0));
    }

    /** Angled mode: a turn in place to a heading in degrees. Both turns (to the aim angle and
     * back to 0) cover the same angle, so they get the same time estimate. */
    private PollenSequence.Step aimTurn(String name, double headingDegrees) {
        double power = PollenOdometryDrive.DEFAULT_TURN_POWER;
        return PollenSequence.Step.turnTo(name, headingDegrees, power,
                PollenOdometryDrive.turnPlannedSeconds(aimDegrees(), power),
                PollenOdometryDrive.turnTimeoutSeconds(aimDegrees(), power));
    }
}
