package org.firstinspires.ftc.teamcode.pollen;

/** Hardware-independent, nonblocking autonomous sequence. Times are seconds.
 * Each step's clock starts on the first loop that runs it (when its motion is first
 * commanded), so a slow loop between steps cannot use up a step. A loop gap longer
 * than MAX_DRIVE_LOOP_GAP_SECONDS while the drive is moving stops everything. */
public final class PollenSequence {
    /** Longest loop gap allowed while drive motors hold a nonzero command. The motors keep
     * their last power during a gap, so a longer gap means unplanned, unmeasured travel. */
    public static final double MAX_DRIVE_LOOP_GAP_SECONDS = 0.25;

    public static final class Step {
        public final String name;
        public final double seconds, forward, strafe, turn;
        public final boolean shoot;

        private Step(String name, double seconds, double forward, double strafe,
                     double turn, boolean shoot) {
            if (name == null || !Double.isFinite(seconds) || seconds <= 0
                    || !Double.isFinite(forward) || !Double.isFinite(strafe)
                    || !Double.isFinite(turn) || Math.abs(forward) > 1
                    || Math.abs(strafe) > 1 || Math.abs(turn) > 1) {
                throw new IllegalArgumentException("Invalid route step");
            }
            this.name = name;
            this.seconds = seconds;
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
            this.shoot = shoot;
        }

        public static Step drive(String name, double seconds, double forward,
                                 double strafe, double turn) {
            return new Step(name, seconds, forward, strafe, turn, false);
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

    /** {@code measuredVelocity} is only checked for invalid data (NaN or infinite stops
     * everything); its value never changes when the feed runs. */
    public Output tick(double now, double measuredVelocity, boolean stopRequested) {
        Output out = compute(now, measuredVelocity, stopRequested);
        wasDriving = !out.finished && (out.forward != 0 || out.strafe != 0 || out.turn != 0);
        return out;
    }

    private Output compute(double now, double measuredVelocity, boolean stopRequested) {
        if (stopRequested) return stop("STOPPED");
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
        if (!step.shoot) {
            if (elapsed >= step.seconds) return advance(now);
            out.forward = step.forward;
            out.strafe = step.strafe;
            out.turn = step.turn;
            return out;
        }
        if (elapsed >= spinupSeconds + step.seconds) return advance(now);
        if (!Double.isFinite(measuredVelocity)) return stop("INVALID LAUNCHER VELOCITY - STOPPED");
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
