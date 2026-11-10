package org.firstinspires.ftc.teamcode.pollen;

import java.util.Locale;

/** Hardware-independent, nonblocking autonomous sequence. Times are seconds.
 *
 * A route is a list of steps, run one after another. There are four kinds (see Kind): a timed
 * drive, a shot, an odometry move to a measured end point, and zeroing the odometry. Every loop
 * the OpMode calls tick(...) with the clock (and, for odometry moves, the robot's pose), and gets
 * back an Output: what every motor should do right now.
 *
 * How to read this file: tick() calls compute(). compute() does the checks that apply to every
 * step, then hands over to one small method per kind of step: driveStep, zeroOdometryStep,
 * shootStep and moveStep, in that order from simplest to longest.
 *
 * Each step's clock starts on the first loop that runs it (when its motion is first
 * commanded), so a slow loop between steps cannot use up a step. A loop gap longer
 * than MAX_DRIVE_LOOP_GAP_SECONDS while the drive is moving stops everything.
 *
 * A move that runs out of time far from its end point, or stops making progress, STOPS the whole
 * route (see PollenOdometryDrive.NEAR_ENOUGH_IN and NO_PROGRESS_SECONDS): the next leg is never
 * started from somewhere the plan did not expect. */
public final class PollenSequence {
    /** Longest loop gap allowed while drive motors hold a nonzero command. The motors keep
     * their last power during a gap, so a longer gap means unplanned, unmeasured travel. */
    public static final double MAX_DRIVE_LOOP_GAP_SECONDS = 0.25;

    /** The four kinds of step a route can contain. */
    public enum Kind {
        /** Hold fixed forward/strafe/turn powers for a set time. All zero is a wait. */
        DRIVE,
        /** Launcher alone for the spin-up time, then launcher and feed together. */
        SHOOT,
        /** Drive to an end point measured by the odometry (PollenOdometryDrive). */
        MOVE,
        /** Tell the OpMode to zero the odometry, then hold still for a moment. */
        ZERO_ODOMETRY
    }

    /** One step of a route. Make one with drive(), continuousShoot(), moveTo() or zeroOdometry(). */
    public static final class Step {
        public final Kind kind;
        public final String name;
        /** DRIVE and ZERO_ODOMETRY: how long the step lasts. SHOOT: the feed time, after the
         * spin-up. MOVE: its PLANNED time, used only by the time budget; a move ends when the
         * robot arrives, or gives up at timeoutSeconds. */
        public final double seconds;
        /** DRIVE only: fixed powers from -1 to 1 (forward +, strafe + = right, turn + = clockwise). */
        public final double forward, strafe, turn;
        /** MOVE only: the end point in inches from the last zero (forward, left), the most
         * power to use, and when to give up. */
        public final double targetForwardIn, targetLeftIn, maxPower, timeoutSeconds;

        private Step(Kind kind, String name, double seconds, double forward, double strafe, double turn,
                     double targetForwardIn, double targetLeftIn, double maxPower, double timeoutSeconds) {
            if (name == null || !Double.isFinite(seconds) || seconds <= 0
                    || !Double.isFinite(forward) || !Double.isFinite(strafe)
                    || !Double.isFinite(turn) || Math.abs(forward) > 1
                    || Math.abs(strafe) > 1 || Math.abs(turn) > 1) {
                throw new IllegalArgumentException("Invalid route step");
            }
            if (kind == Kind.MOVE && (!Double.isFinite(targetForwardIn) || !Double.isFinite(targetLeftIn)
                    || !Double.isFinite(maxPower) || maxPower <= 0 || maxPower > 1
                    || !Double.isFinite(timeoutSeconds) || timeoutSeconds < seconds)) {
                throw new IllegalArgumentException("Invalid odometry move");
            }
            this.kind = kind;
            this.name = name;
            this.seconds = seconds;
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
            this.targetForwardIn = targetForwardIn;
            this.targetLeftIn = targetLeftIn;
            this.maxPower = maxPower;
            this.timeoutSeconds = timeoutSeconds;
        }

        /** Hold these drive powers for {@code seconds}. All three zero is a wait. */
        public static Step drive(String name, double seconds, double forward,
                                 double strafe, double turn) {
            return new Step(Kind.DRIVE, name, seconds, forward, strafe, turn, 0, 0, 0, seconds);
        }

        /** Drive to an end point measured by odometry, at up to {@code maxPower}. */
        public static Step moveTo(String name, double targetForwardIn, double targetLeftIn,
                                  double maxPower, double plannedSeconds, double timeoutSeconds) {
            return new Step(Kind.MOVE, name, plannedSeconds, 0, 0, 0,
                    targetForwardIn, targetLeftIn, maxPower, timeoutSeconds);
        }

        /** Zero the odometry here (x = y = heading = 0), then hold still for {@code seconds}. */
        public static Step zeroOdometry(double seconds) {
            return new Step(Kind.ZERO_ODOMETRY, "ZERO ODOMETRY", seconds, 0, 0, 0, 0, 0, 0, seconds);
        }

        /** Launcher on for the sequence's spin-up time, then feed for {@code seconds}. */
        public static Step continuousShoot(double seconds) {
            return new Step(Kind.SHOOT, "SHOOT", seconds, 0, 0, 0, 0, 0, 0, seconds);
        }
    }

    /** What every motor should do on this loop. Anything not set is zero (off). */
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
    /** Which step is running, and whether its clock has started. */
    private int index;
    private boolean stepActive;
    /** True if the last loop commanded the drive motors (for the loop-gap check). */
    private boolean wasDriving;
    /** Odometry move: last loop's position, to work out how fast the robot is moving. */
    private double lastMoveX = Double.NaN, lastMoveY = Double.NaN;
    /** Odometry move, stuck check: where and when the robot last made real progress. */
    private double progressX = Double.NaN, progressY = Double.NaN, progressAt;
    private double startedAt, stepStartedAt, previousTime;
    private boolean started, finished;
    private String result = "READY";
    private String warning = "";

    /** Drive-only route: callers do not need irrelevant launcher settings. */
    public static PollenSequence park(Step[] steps, double deadline, double startDelay) {
        if (steps != null) for (Step step : steps) {
            if (step != null && step.kind == Kind.SHOOT) throw new IllegalArgumentException("Park route cannot shoot");
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
            if (step != null && step.kind == Kind.SHOOT && targetVelocity <= 0)
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
            seconds += step.kind == Kind.SHOOT ? spinupSeconds + step.seconds : step.seconds;
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

        // Checks that apply to every kind of step.
        if (!Double.isFinite(now) || now < previousTime) return stop("INVALID CLOCK");
        double sinceLastLoop = now - previousTime;
        previousTime = now;
        if (wasDriving && sinceLastLoop > MAX_DRIVE_LOOP_GAP_SECONDS) {
            warning = String.format(Locale.US, "Loop paused %.2f s while driving", sinceLastLoop);
            return stop("LOOP STALL WHILE DRIVING - STOPPED");
        }
        if (now - startedAt >= deadline) return stop("TIME LIMIT - STOPPED");
        if (now - startedAt < startDelay) return new Output("START DELAY");

        // Run the current step.
        Step step = steps[index];
        if (!stepActive) beginStep(now);
        double elapsed = now - stepStartedAt; // seconds since this step began
        if (step.kind == Kind.SHOOT) return shootStep(step, elapsed);
        if (step.kind == Kind.MOVE) return moveStep(step, now, elapsed, sinceLastLoop, x, y, heading);
        if (step.kind == Kind.ZERO_ODOMETRY) return zeroOdometryStep(step, elapsed);
        return driveStep(step, elapsed); // Kind.DRIVE
    }

    /** Start this step's clock now, on the loop that first commands it. */
    private void beginStep(double now) {
        stepActive = true;
        stepStartedAt = now;
        lastMoveX = lastMoveY = Double.NaN;
        progressX = progressY = Double.NaN;
    }

    /** DRIVE: the same powers every loop until the time is up. */
    private Output driveStep(Step step, double elapsed) {
        if (elapsed >= step.seconds) return advance();
        Output out = new Output(step.name);
        out.forward = step.forward;
        out.strafe = step.strafe;
        out.turn = step.turn;
        return out;
    }

    /** ZERO_ODOMETRY: ask the OpMode to zero on the first loop, then hold still (all outputs
     * zero) so no later loop acts on a pose that was read before the zero. */
    private Output zeroOdometryStep(Step step, double elapsed) {
        if (elapsed >= step.seconds) return advance();
        Output out = new Output(step.name);
        out.zeroOdometry = elapsed == 0; // first loop of this step only
        return out;
    }

    /** SHOOT: launcher alone for the spin-up time, then launcher and feed for the step's seconds. */
    private Output shootStep(Step step, double elapsed) {
        if (elapsed >= spinupSeconds + step.seconds) return advance();
        Output out = new Output(step.name);
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

    /** MOVE: drive toward the end point until the robot has arrived. Two things end it early:
     * running out of time, and not moving while the wheels are driven. */
    private Output moveStep(Step step, double now, double elapsed, double sinceLastLoop,
                            double x, double y, double heading) {
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading))
            return stop("NO ODOMETRY POSE - STOPPED");

        // How fast is the robot moving? Compare this loop's position with the last loop's.
        // On a move's first loop there is no last position yet, so the speed is unknown
        // (treated as "not stopped").
        boolean haveSpeed = !Double.isNaN(lastMoveX) && sinceLastLoop > 0;
        double vx = haveSpeed ? (x - lastMoveX) / sinceLastLoop : 0;
        double vy = haveSpeed ? (y - lastMoveY) / sinceLastLoop : 0;
        double speed = haveSpeed ? Math.hypot(vx, vy) : Double.POSITIVE_INFINITY;
        lastMoveX = x;
        lastMoveY = y;

        // Ask the drive controller for this loop's powers and the distance left.
        double powerLimit = PollenOdometryDrive.rampedPower(elapsed, step.maxPower);
        PollenOdometryDrive.Command drive = PollenOdometryDrive.command(
                step.targetForwardIn, step.targetLeftIn, powerLimit, x, y, heading, vx, vy);

        // 1. Arrived? Only when inside the tolerance AND stopped, so it can't coast past and stay there.
        boolean insideTolerance = drive.inchesToGo <= PollenOdometryDrive.POSITION_TOLERANCE_IN;
        boolean stopped = speed <= PollenOdometryDrive.SETTLED_IN_PER_SECOND;
        if (insideTolerance && stopped) return advance();

        // 2. Out of time?
        if (elapsed >= step.timeoutSeconds) return moveOutOfTime(step, drive.inchesToGo, elapsed);

        // 3. Stuck?
        if (stuck(step, now, x, y, drive)) return stop(step.name + " NOT MOVING - STOPPED");

        // 4. Still on the way: pass the controller's powers on.
        Output out = new Output(step.name);
        out.forward = drive.forward;
        out.strafe = drive.strafe;
        out.turn = drive.turn;
        out.inchesToGo = drive.inchesToGo;
        return out;
    }

    /** A move has used up its give-up time. Almost there (still settling): hand over to the
     * next step. Farther away: stop the whole route. */
    private Output moveOutOfTime(Step step, double inchesToGo, double elapsed) {
        if (inchesToGo <= PollenOdometryDrive.NEAR_ENOUGH_IN) {
            warning = String.format(Locale.US, "%s ended %.1f in short after %.1f s (close enough)",
                    step.name, inchesToGo, elapsed);
            return advance();
        }
        warning = String.format(Locale.US, "%s ran out of time %.1f in short after %.1f s",
                step.name, inchesToGo, elapsed);
        return stop(step.name + " TIMED OUT - STOPPED");
    }

    /** Stuck check for a move: true (with a warning) when the wheels have been driven for
     * NO_PROGRESS_SECONDS but the odometry has moved less than NO_PROGRESS_MIN_IN. */
    private boolean stuck(Step step, double now, double x, double y, PollenOdometryDrive.Command drive) {
        boolean almostThere = drive.inchesToGo <= PollenOdometryDrive.NEAR_ENOUGH_IN;
        boolean firstLook = Double.isNaN(progressX);
        boolean madeProgress = !firstLook
                && Math.hypot(x - progressX, y - progressY) >= PollenOdometryDrive.NO_PROGRESS_MIN_IN;
        if (!drive.driving() || almostThere || firstLook || madeProgress) {
            // Progress (or coasting, or almost there): restart the stuck clock from here.
            progressX = x;
            progressY = y;
            progressAt = now;
            return false;
        }
        if (now - progressAt < PollenOdometryDrive.NO_PROGRESS_SECONDS) return false;
        warning = String.format(Locale.US,
                "%s: wheels driven but odometry moved < %.1f in in %.2f s, %.1f in short",
                step.name, PollenOdometryDrive.NO_PROGRESS_MIN_IN, now - progressAt, drive.inchesToGo);
        return true;
    }

    /** This step is done: line up the next one, or finish. Outputs are zero for this one loop. */
    private Output advance() {
        index++;
        stepActive = false; // the next step's clock starts when it first runs
        if (index == steps.length) return stop("DONE - all outputs zero");
        return new Output("NEXT: " + steps[index].name);
    }

    /** End the run: from now on every tick returns all outputs zero and this reason. */
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
    /** Final (or current) result line, e.g. "DONE - all outputs zero" or "LEG 2 CLEAR HIVE TIMED OUT - STOPPED". */
    public String result() { return result; }

    /** Mecanum mixer: turns forward/strafe/turn into four wheel powers, scaled down together if
     * any would be over 1. Output order: frontLeft, frontRight, backLeft, backRight. */
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
