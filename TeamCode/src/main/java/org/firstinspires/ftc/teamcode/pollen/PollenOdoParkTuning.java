package org.firstinspires.ftc.teamcode.pollen;

/** INIT controls for Option A with odometry. Same buttons as PollenParkTuning, but LEFT/RIGHT
 * changes the distance driven (cm) instead of the drive time. Pure Java, no hardware commands. */
public final class PollenOdoParkTuning {
    public static final double MIN_POWER = 0.10, MAX_POWER = PollenOdometryDrive.MAX_LEG_POWER, POWER_INCREMENT = 0.05;
    public static final double MIN_CM = 5, MAX_CM = 150, CM_INCREMENT = 5;
    public static final double DELAY_INCREMENT = 0.5;
    /** Default distance: the mat model's estimate for the tested timed A (2.0 s at 0.20 power),
     * PollenDriveCalibration.forwardDistanceCm = 55.3 cm, rounded to the 5 cm step. 0.20 is below
     * the measured powers (extrapolated), so check it on the robot. */
    public static final double DEFAULT_CM = 55;

    /** Only power and distance carry over between OpMode instances; delay resets each INIT. */
    public static final class Snapshot {
        public final double power, cm;
        private Snapshot(double power, double cm) {
            this.power = power;
            this.cm = cm;
        }
    }

    private final double defaultPower, defaultDelay, deadline, holdSeconds;
    private double power, cm, delay;
    private boolean locked, previousUp, previousDown, previousLeft, previousRight;
    private boolean previousLessPower, previousMorePower, previousReset;

    public PollenOdoParkTuning(double defaultPower, double defaultDelay, double deadline,
                               double holdSeconds, Snapshot remembered) {
        if (!Double.isFinite(defaultPower) || defaultPower < MIN_POWER || defaultPower > MAX_POWER
                || !Double.isFinite(defaultDelay) || defaultDelay < 0
                || !Double.isFinite(holdSeconds) || holdSeconds <= 0 || !Double.isFinite(deadline)
                || routeSeconds(MAX_CM, MIN_POWER, holdSeconds) >= deadline - 1) {
            throw new IllegalArgumentException("Invalid odometry parking defaults or time budget");
        }
        this.defaultPower = defaultPower;
        this.defaultDelay = defaultDelay;
        this.deadline = deadline;
        this.holdSeconds = holdSeconds;
        power = remembered == null ? defaultPower : remembered.power;
        cm = remembered == null ? DEFAULT_CM : remembered.cm;
        delay = defaultDelay;
        clampDelay();
    }

    /** UP/DOWN delay, LEFT/RIGHT distance, L1/R1 power, Triangle resets. Tap and release. */
    public void update(boolean up, boolean down, boolean left, boolean right,
                       boolean lessPower, boolean morePower, boolean reset) {
        if (locked) return;
        if (reset) {
            if (!previousReset) {
                power = defaultPower;
                cm = DEFAULT_CM;
                delay = defaultDelay;
            }
        } else {
            if (right && !left && !previousRight) cm = step(cm, CM_INCREMENT, MIN_CM, MAX_CM);
            if (left && !right && !previousLeft) cm = step(cm, -CM_INCREMENT, MIN_CM, MAX_CM);
            if (morePower && !lessPower && !previousMorePower) power = step(power, POWER_INCREMENT, MIN_POWER, MAX_POWER);
            if (lessPower && !morePower && !previousLessPower) power = step(power, -POWER_INCREMENT, MIN_POWER, MAX_POWER);
            if (up && !down && !previousUp) delay += DELAY_INCREMENT;
            if (down && !up && !previousDown) delay -= DELAY_INCREMENT;
        }
        clampDelay();
        previousUp = up;
        previousDown = down;
        previousLeft = left;
        previousRight = right;
        previousLessPower = lessPower;
        previousMorePower = morePower;
        previousReset = reset;
    }

    /** Zero, drive, hold: expected seconds after the start delay. */
    private static double routeSeconds(double cm, double power, double hold) {
        return PollenOdometryDrive.ZERO_SETTLE_SECONDS + PollenOdometryDrive.plannedSeconds(cm, power) + hold;
    }

    private static double step(double value, double amount, double min, double max) {
        return Math.max(min, Math.min(max, Math.round((value + amount) * 100.0) / 100.0));
    }

    private void clampDelay() {
        delay = Math.max(0, Math.min(maximumDelaySeconds(), Math.round(delay * 2.0) / 2.0));
    }

    public double maximumDelaySeconds() {
        double max = 0;
        while (max + DELAY_INCREMENT <= 20
                && routeSeconds(cm, power, holdSeconds) + max + DELAY_INCREMENT < deadline - 1) {
            max += DELAY_INCREMENT;
        }
        return max;
    }

    /** Zero the odometry, drive straight cm (route forward: see PollenFacing), then hold still. */
    public PollenSequence sequence() {
        PollenSequence.Step[] steps = {
                PollenSequence.Step.zeroOdometry(PollenOdometryDrive.ZERO_SETTLE_SECONDS),
                PollenSequence.Step.moveTo("DRIVE TO PARK", cm / PollenOdometryDrive.CM_PER_INCH, 0, power,
                        PollenOdometryDrive.plannedSeconds(cm, power), PollenOdometryDrive.timeoutSeconds(cm, power)),
                PollenSequence.Step.drive("PARK / HOLD", holdSeconds, 0, 0, 0)
        };
        return PollenSequence.park(steps, deadline, delay);
    }

    public double plannedSeconds() { return delay + routeSeconds(cm, power, holdSeconds); }
    public void lock() { locked = true; }
    public boolean locked() { return locked; }
    public double power() { return power; }
    public double cm() { return cm; }
    public double delaySeconds() { return delay; }
    public Snapshot snapshot() { return new Snapshot(power, cm); }
}
