package org.firstinspires.ftc.teamcode.pollen;

/** Hardware-independent, nonblocking autonomous sequence. Times are seconds.
 * Steps are timed drives, shots, or (odometry autos) moves to a measured end point; see
 * PollenOdometryDrive. Move steps need the pose every loop: call tick(now, x, y, heading).
 * Each step's clock starts on the first loop that runs it (when its motion is first
 * commanded), so a slow loop between steps cannot use up a step. A loop gap longer
 * than MAX_DRIVE_LOOP_GAP_SECONDS while the drive is moving stops everything. */
public final class PollenSequence {
    /** Longest loop gap allowed while drive motors hold a nonzero command. The motors keep
     * their last power during a gap, so a longer gap means unplanned, unmeasured travel. */
    public static final double MAX_DRIVE_LOOP_GAP_SECONDS = 0.25;

    public static final class Step {
        public final String name;
        /** For a move step, seconds is its PLANNED time (used by the budget); timeoutSeconds is
         * when it gives up. For every other step the two are equal. */
        public final double seconds, forward, strafe, turn;
        public final boolean shoot;
        /** Odometry move: end point in inches from the last zero (x forward, y left). */
        public final boolean move;
        public final double targetForwardIn, targetLeftIn, maxPower, timeoutSeconds;
        /** Tells the OpMode to zero the odometry on this step's first loop; outputs stay zero. */
        public final boolean zeroOdometry;

        private Step(String name, double seconds, double forward, double strafe,
                     double turn, boolean shoot) {
            this(name, seconds, forward, strafe, turn, shoot, false, 0, 0, 0, seconds, false);
        }

        private Step(String name, double seconds, double forward, double strafe, double turn,
                     boolean shoot, boolean move, double targetForwardIn, double targetLeftIn,
                     double maxPower, double timeoutSeconds, boolean zeroOdometry) {
            if (name == null || !Double.isFinite(seconds) || seconds <= 0
                    || !Double.isFinite(forward) || !Double.isFinite(strafe)
                    || !Double.isFinite(turn) || Math.abs(forward) > 1
                    || Math.abs(strafe) > 1 || Math.abs(turn) > 1) {
                throw new IllegalArgumentException("Invalid route step");
            }
            if (move && (!Double.isFinite(targetForwardIn) || !Double.isFinite(targetLeftIn)
                    || !Double.isFinite(maxPower) || maxPower <= 0 || maxPower > 1
                    || !Double.isFinite(timeoutSeconds) || timeoutSeconds < seconds)) {
                throw new IllegalArgumentException("Invalid odometry move");
            }
            this.name = name;
            this.seconds = seconds;
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
            this.shoot = shoot;
            this.move = move;
            this.targetForwardIn = targetForwardIn;
            this.targetLeftIn = targetLeftIn;
            this.maxPower = maxPower;
            this.timeoutSeconds = timeoutSeconds;
            this.zeroOdometry = zeroOdometry;
        }

        public static Step drive(String name, double seconds, double forward,
                                 double strafe, double turn) {
            return new Step(name, seconds, forward, strafe, turn, false);
        }

        /** Drive to an end point measured by odometry, at up to {@code maxPower}. */
        public static Step moveTo(String name, double targetForwardIn, double targetLeftIn,
                                  double maxPower, double plannedSeconds, double timeoutSeconds) {
            return new Step(name, plannedSeconds, 0, 0, 0, false, true,
                    targetForwardIn, targetLeftIn, maxPower, timeoutSeconds, false);
        }

        /** Zero the odometry here (x = y = heading = 0), then hold still for {@code seconds}. */
        public static Step zeroOdometry(double seconds) {
            return new Step("ZERO ODOMETRY", seconds, 0, 0, 0, false, false, 0, 0, 0, seconds, true);
        }

        /** Launcher on for the sequence's spin-up time, then feed for {@code seconds}. */
        public static Step continuousShoot(double seconds) {
            return new Step("SHOOT", seconds, 0, 0, 0, true);
        }
    }

    public static final class Output {
        public String phase;
        public double forward, strafe, turn, launcherVelocity, feederPower, activeFeedSeconds;
        public boolean finished;
        /** True on the one loop the OpMode must zero its odometry. */
        public boolean zeroOdometry;
        /** Odometry move: inches still to go to this leg's end point (NaN otherwise). */
        public double inchesToGo = Double.NaN;
        private Output(String phase) { this.phase = phase; }
    }

    private final Step[] steps;
    private final double targetVelocity, spinupSeconds, deadline, startDelay;
    private int index;
    private boolean stepActive, wasDriving;
    private double startedAt, stepStartedAt, previousTime;
    private boolean started, finished;
    private String result = "READY";
    private String warning = "";

    /** Drive-only route: callers do not need irrelevant launcher settings. */
    public static PollenSequence park(Step[] steps, double deadline, double startDelay) {
        if (steps != null) for (Step step : steps) {
            if (step != null && step.shoot) throw new IllegalArgumentException("Park route cannot shoot");
        }
        return new PollenSequence(steps, 0, 0, deadline, startDelay);
    }

    /** A shoot step runs the launcher at {@code targetVelocity} for {@code spinupSeconds},
     * then keeps it on and feeds for the step's seconds. Timing is fixed: measured launcher
     * speed never delays, skips or stops feeding. */
    public PollenSequence(Step[] steps, double targetVelocity, double spinupSeconds,
                          double deadline, double startDelay) {
        if (steps == null || steps.length == 0 || !Double.isFinite(targetVelocity) || targetVelocity < 0
                || !Double.isFinite(spinupSeconds) || spinupSeconds < 0
                || !Double.isFinite(deadline) || deadline <= 0
                || !Double.isFinite(startDelay) || startDelay < 0) {
            throw new IllegalArgumentException("Invalid sequence settings");
        }
        for (Step step : steps) {
            if (step != null && step.shoot && targetVelocity <= 0)
                throw new IllegalArgumentException("A shooting step needs a launcher velocity");
        }
        // Leave room for transition loops; the independent deadline always stops outputs.
        if (startDelay + worstCaseDuration(steps, spinupSeconds) >= deadline - 1) {
            throw new IllegalArgumentException("Route exceeds autonomous time budget");
        }
        this.steps = steps.clone();
        this.targetVelocity = targetVelocity;
        this.spinupSeconds = spinupSeconds;
        this.deadline = deadline;
        this.startDelay = startDelay;
    }

    /** Route duration, excluding start delay and loop-transition reserve. */
    public static double worstCaseDuration(Step[] steps, double spinupSeconds) {
        if (steps == null || steps.length == 0 || !Double.isFinite(spinupSeconds) || spinupSeconds < 0) {
            throw new IllegalArgumentException("Invalid route budget");
        }
        double seconds = 0;
        for (Step step : steps) {
            if (step == null) throw new IllegalArgumentException("Null step");
            seconds += step.shoot ? spinupSeconds + step.seconds : step.seconds;
        }
        return seconds;
    }

    public void start(double now) {
        if (!Double.isFinite(now)) throw new IllegalArgumentException("Invalid time");
        index = 0;
        startedAt = previousTime = now;
        stepActive = wasDriving = false;
        stepStartedAt = Double.NaN;
        finished = false;
        started = true;
        warning = "";
        result = "RUNNING";
    }

    /** Call once per loop with the match clock; returns what every motor should do right now.
     * The launcher's measured speed is not an input: autonomous feeds on fixed times. */
    public Output tick(double now) {
        return tick(now, Double.NaN, Double.NaN, Double.NaN);
    }

    /** Same, with the odometry pose (inches, radians, from the last zero; x forward, y left,
     * heading counterclockwise). A move step with no valid pose stops everything. */
    public Output tick(double now, double x, double y, double heading) {
        Output out = compute(now, x, y, heading);
        wasDriving = !out.finished && (out.forward != 0 || out.strafe != 0 || out.turn != 0);
        return out;
    }

    private Output compute(double now, double x, double y, double heading) {
        if (finished) return terminal();
        if (!started) return new Output("READY - waiting for START");
        if (!Double.isFinite(now) || now < previousTime) return stop("INVALID CLOCK");
        double elapsedSinceTick = now - previousTime;
        previousTime = now;
        if (wasDriving && elapsedSinceTick > MAX_DRIVE_LOOP_GAP_SECONDS) {
            warning = String.format(java.util.Locale.US, "Loop paused %.2f s while driving", elapsedSinceTick);
            return stop("LOOP STALL WHILE DRIVING - STOPPED");
        }
        if (now - startedAt >= deadline) return stop("TIME LIMIT - STOPPED");
        if (now - startedAt < startDelay) return new Output("START DELAY");
        Step step = steps[index];
        if (!stepActive) {
            // Start this step's clock now, on the loop that first commands it.
            stepActive = true;
            stepStartedAt = now;
        }
        double elapsed = now - stepStartedAt;
        Output out = new Output(step.name);
        if (step.zeroOdometry) {
            if (elapsed >= step.seconds) return advance(now);
            out.zeroOdometry = elapsed == 0; // first loop of this step only
            return out;
        }
        if (step.move) {
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading))
                return stop("NO ODOMETRY POSE - STOPPED");
            double[] cmd = PollenOdometryDrive.command(step.targetForwardIn, step.targetLeftIn,
                    step.maxPower, x, y, heading);
            if (cmd[3] <= PollenOdometryDrive.POSITION_TOLERANCE_IN) return advance(now);
            if (elapsed >= step.timeoutSeconds) {
                warning = String.format(java.util.Locale.US, "%s gave up %.1f in short after %.1f s",
                        step.name, cmd[3], elapsed);
                return advance(now);
            }
            out.forward = cmd[0];
            out.strafe = cmd[1];
            out.turn = cmd[2];
            out.inchesToGo = cmd[3];
            return out;
        }
        if (!step.shoot) {
            if (elapsed >= step.seconds) return advance(now);
            out.forward = step.forward;
            out.strafe = step.strafe;
            out.turn = step.turn;
            return out;
        }
        if (elapsed >= spinupSeconds + step.seconds) return advance(now);
        out.launcherVelocity = targetVelocity;
        if (elapsed < spinupSeconds) {
            out.phase = "SPINUP - launcher only";
        } else {
            out.phase = "FEED";
            out.feederPower = 1;
            out.activeFeedSeconds = elapsed - spinupSeconds;
        }
        return out;
    }

    private Output advance(double now) {
        index++;
        stepActive = false; // the next step's clock starts when it first runs
        if (index == steps.length) return stop("DONE - all outputs zero");
        return new Output("NEXT: " + steps[index].name); // Zero outputs between steps.
    }

    public Output stop(String reason) {
        finished = true;
        result = reason;
        return terminal();
    }

    private Output terminal() {
        Output out = new Output(result);
        out.finished = true;
        return out;
    }

    public String warning() { return warning; }

    /** Output order: frontLeft, frontRight, backLeft, backRight. */
    public static double[] wheelPowers(Output out) {
        double[] powers = {out.forward + out.strafe + out.turn,
                out.forward - out.strafe - out.turn,
                out.forward - out.strafe + out.turn,
                out.forward + out.strafe - out.turn};
        double max = 1;
        for (double power : powers) max = Math.max(max, Math.abs(power));
        for (int i = 0; i < powers.length; i++) powers[i] /= max;
        return powers;
    }
}
