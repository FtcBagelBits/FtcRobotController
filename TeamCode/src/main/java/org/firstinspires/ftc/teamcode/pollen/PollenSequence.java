package org.firstinspires.ftc.teamcode.pollen;

import java.util.Locale;

/** Hardware-independent, nonblocking autonomous sequence. Times are seconds.
 *
 * A route is a list of steps, run one after another. There are five kinds (see Kind): a timed
 * drive, a shot, an odometry move to a measured end point, a turn in place to a heading, and
 * zeroing the odometry. Every loop the OpMode calls tick(...) with the clock and its sensor
 * readings (the robot's pose for moves and turns, the launcher's speed for shots), and gets back
 * an Output: what every motor should do right now.
 *
 * How to read this file: tick() calls compute(). compute() does the checks that apply to every
 * step, then hands over to one small method per kind of step: driveStep, zeroOdometryStep,
 * shootStep, moveStep and turnStep, in that order from simplest to longest.
 *
 * Each step's clock starts on the first loop that runs it (when its motion is first
 * commanded), so a slow loop between steps cannot use up a step. A loop gap longer
 * than MAX_DRIVE_LOOP_GAP_SECONDS while the drive is moving stops everything.
 *
 * A move that runs out of time far from its end point, or stops making progress, STOPS the whole
 * route (see PollenOdometryDrive.NEAR_ENOUGH_IN and NO_PROGRESS_SECONDS): the next leg is never
 * started from somewhere the plan did not expect. A turn follows the same rule. */
public final class PollenSequence {
    /** Longest loop gap allowed while drive motors hold a nonzero command. The motors keep
     * their last power during a gap, so a longer gap means unplanned, unmeasured travel. */
    public static final double MAX_DRIVE_LOOP_GAP_SECONDS = 0.25;

    /** The five kinds of step a route can contain. */
    public enum Kind {
        /** Hold fixed forward/strafe/turn powers for a set time. All zero is a wait. */
        DRIVE,
        /** Launcher alone until it is up to speed (or the spin-up time is over), then launcher
         * and feed together: the feed rule in PollenFeedRule. */
        SHOOT,
        /** Drive to an end point measured by the odometry (PollenOdometryDrive). */
        MOVE,
        /** Turn in place to a heading measured by the odometry. EXPERIMENTAL. */
        TURN,
        /** Tell the OpMode to zero the odometry, then hold still for a moment. */
        ZERO_ODOMETRY
    }

    /** One step of a route. Make one with drive(), continuousShoot(), moveTo(), turnTo() or zeroOdometry(). */
    public static final class Step {
        public final Kind kind;
        public final String name;
        /** DRIVE and ZERO_ODOMETRY: how long the step lasts. SHOOT: the feed time, counted from
         * when feeding starts. MOVE and TURN: the PLANNED time, used only by the time budget; a
         * move or turn ends when the robot arrives, or gives up at timeoutSeconds. */
        public final double seconds;
        /** DRIVE only: fixed powers from -1 to 1 (forward +, strafe + = right, turn + = clockwise). */
        public final double forward, strafe, turn;
        /** MOVE only: the end point in inches from the last zero (forward, left). */
        public final double targetForwardIn, targetLeftIn;
        /** MOVE and TURN: the most power to use, and when to give up. */
        public final double maxPower, timeoutSeconds;
        /** TURN only: the heading to turn to, in radians from the last zero (+ = counterclockwise). */
        public final double targetHeading;
        /** SHOOT only: true = feeding may start as soon as the launcher is up to speed (the
         * normal feed rule). False = always wait the whole spin-up time: the Supporting Shooter
         * uses that to feed at the time agreed with the partner, never sooner. */
        public final boolean feedWhenUpToSpeed;

        private Step(Kind kind, String name, double seconds, double forward, double strafe, double turn,
                     double targetForwardIn, double targetLeftIn, double maxPower, double timeoutSeconds,
                     double targetHeading, boolean feedWhenUpToSpeed) {
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
            if (kind == Kind.TURN && (!Double.isFinite(targetHeading)
                    || !Double.isFinite(maxPower) || maxPower <= 0 || maxPower > 1
                    || !Double.isFinite(timeoutSeconds) || timeoutSeconds < seconds)) {
                throw new IllegalArgumentException("Invalid turn");
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
            this.targetHeading = targetHeading;
            this.feedWhenUpToSpeed = feedWhenUpToSpeed;
        }

        /** Hold these drive powers for {@code seconds}. All three zero is a wait. */
        public static Step drive(String name, double seconds, double forward,
                                 double strafe, double turn) {
            return new Step(Kind.DRIVE, name, seconds, forward, strafe, turn, 0, 0, 0, seconds, 0, false);
        }

        /** Drive to an end point measured by odometry, at up to {@code maxPower}. */
        public static Step moveTo(String name, double targetForwardIn, double targetLeftIn,
                                  double maxPower, double plannedSeconds, double timeoutSeconds) {
            return new Step(Kind.MOVE, name, plannedSeconds, 0, 0, 0,
                    targetForwardIn, targetLeftIn, maxPower, timeoutSeconds, 0, false);
        }

        /** Turn in place to {@code targetHeadingDegrees} from the last zero (+ = counterclockwise,
         * the same way leg angles are + = left), at up to {@code maxPower}. EXPERIMENTAL. */
        public static Step turnTo(String name, double targetHeadingDegrees, double maxPower,
                                  double plannedSeconds, double timeoutSeconds) {
            return new Step(Kind.TURN, name, plannedSeconds, 0, 0, 0,
                    0, 0, maxPower, timeoutSeconds, Math.toRadians(targetHeadingDegrees), false);
        }

        /** Zero the odometry here (x = y = heading = 0), then hold still for {@code seconds}. */
        public static Step zeroOdometry(double seconds) {
            return new Step(Kind.ZERO_ODOMETRY, "ZERO ODOMETRY", seconds, 0, 0, 0, 0, 0, 0, seconds, 0, false);
        }

        /** Launcher on; feed for {@code seconds} once the launcher is up to speed or the
         * sequence's spin-up time is over, whichever comes first (PollenFeedRule). */
        public static Step continuousShoot(double seconds) {
            return new Step(Kind.SHOOT, "SHOOT", seconds, 0, 0, 0, 0, 0, 0, seconds, 0, true);
        }

        /** Launcher on; feed for {@code seconds} only after the WHOLE spin-up time, even if the
         * launcher is up to speed sooner. For a shot whose time is agreed with a partner. */
        public static Step shootAtSetTime(double seconds) {
            return new Step(Kind.SHOOT, "SHOOT", seconds, 0, 0, 0, 0, 0, 0, seconds, 0, false);
        }
    }

    /** What every motor should do on this loop. Anything not set is zero (off). */
    public static final class Output {
        public String phase;
        public double forward, strafe, turn, launcherVelocity, feederPower, activeFeedSeconds;
        /** While feeding: which half of the feed rule started it ("up to speed" or "spin-up timer"). */
        public String feedStartedBy = "";
        public boolean finished;
        /** True on the one loop the OpMode must zero its odometry. */
        public boolean zeroOdometry;
        /** Odometry move: inches still to go to this leg's end point (NaN otherwise). */
        public double inchesToGo = Double.NaN;
        /** Turn: degrees still to go to the target heading (NaN otherwise). */
        public double degreesToGo = Double.NaN;
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
    /** Turn: last loop's heading (for the turn rate), and the heading at the last real progress. */
    private double lastTurnHeading = Double.NaN, progressHeading = Double.NaN;
    /** Shot: seconds into the step when feeding started (NaN = not feeding yet), and why. */
    private double feedStartedAt = Double.NaN;
    private String feedStartedBy = "";
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

    /** A shoot step runs the launcher at {@code targetVelocity}. Feeding starts when the launcher
     * is up to speed or after {@code spinupSeconds}, whichever comes first, and then lasts the
     * step's seconds. The measured speed can only make feeding start SOONER than the spin-up
     * time: it never delays, skips or stops the feed. */
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
     * This short form has no sensor readings: no pose (so moves and turns stop the run) and no
     * launcher speed (so a shot feeds when its spin-up time is over). */
    public Output tick(double now) {
        return tick(now, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
    }

    /** Same, with the odometry pose but no launcher speed. */
    public Output tick(double now, double x, double y, double heading) {
        return tick(now, x, y, heading, Double.NaN);
    }

    /** The full form: the clock, the odometry pose (inches, radians, from the last zero; x
     * forward, y left, heading counterclockwise) and the launcher's measured speed in ticks per
     * second. NaN means "no reading": a move or turn with no valid pose stops everything; a
     * shot with no speed reading feeds when its spin-up time is over. */
    public Output tick(double now, double x, double y, double heading, double launcherVelocity) {
        Output out = compute(now, x, y, heading, launcherVelocity);
        wasDriving = !out.finished && (out.forward != 0 || out.strafe != 0 || out.turn != 0);
        return out;
    }

    private Output compute(double now, double x, double y, double heading, double launcherVelocity) {
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
        if (step.kind == Kind.SHOOT) return shootStep(step, elapsed, launcherVelocity);
        if (step.kind == Kind.MOVE) return moveStep(step, now, elapsed, sinceLastLoop, x, y, heading);
        if (step.kind == Kind.TURN) return turnStep(step, now, elapsed, sinceLastLoop, heading);
        if (step.kind == Kind.ZERO_ODOMETRY) return zeroOdometryStep(step, elapsed);
        return driveStep(step, elapsed); // Kind.DRIVE
    }

    /** Start this step's clock now, on the loop that first commands it. */
    private void beginStep(double now) {
        stepActive = true;
        stepStartedAt = now;
        lastMoveX = lastMoveY = Double.NaN;
        progressX = progressY = Double.NaN;
        lastTurnHeading = progressHeading = Double.NaN;
        feedStartedAt = Double.NaN;
        feedStartedBy = "";
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

    /** SHOOT: launcher alone until feeding may start, then launcher and feed for the step's
     * seconds. When feeding may start is THE FEED RULE (PollenFeedRule), the same one TeleOp uses. */
    private Output shootStep(Step step, double elapsed, double launcherVelocity) {
        boolean feeding = !Double.isNaN(feedStartedAt);
        if (!feeding) {
            boolean upToSpeed = step.feedWhenUpToSpeed && PollenFeedRule.upToSpeed(launcherVelocity, targetVelocity);
            boolean spinUpTimeIsOver = PollenFeedRule.spinUpTimeIsOver(elapsed, spinupSeconds);
            if (upToSpeed || spinUpTimeIsOver) {
                // Remember that feeding has started: from here on a speed dip changes nothing.
                feeding = true;
                feedStartedBy = spinUpTimeIsOver ? "spin-up timer" : "up to speed";
                // Started by the timer: count the feed from the moment the spin-up time ended,
                // not from this loop a few milliseconds later, so the shot ends when planned.
                feedStartedAt = spinUpTimeIsOver ? spinupSeconds : elapsed;
            }
        }
        Output out = new Output("SPINUP - launcher only");
        out.launcherVelocity = targetVelocity;
        if (!feeding) return out;

        double fedSeconds = elapsed - feedStartedAt;
        if (fedSeconds >= step.seconds) return advance();
        out.phase = "FEED";
        out.feederPower = 1;
        out.activeFeedSeconds = fedSeconds;
        out.feedStartedBy = feedStartedBy;
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

    /** TURN: turn in place toward the target heading until the robot is there and has stopped
     * turning. Like a move, two things end it early: running out of time, and not turning while
     * the wheels are driven. EXPERIMENTAL: not yet tried on the robot. */
    private Output turnStep(Step step, double now, double elapsed, double sinceLastLoop, double heading) {
        if (!Double.isFinite(heading)) return stop("NO ODOMETRY POSE - STOPPED");

        // How fast is the robot turning? Compare this loop's heading with the last loop's.
        boolean haveRate = !Double.isNaN(lastTurnHeading) && sinceLastLoop > 0;
        double turnRate = haveRate ? PollenOdometryDrive.wrap(heading - lastTurnHeading) / sinceLastLoop : 0;
        lastTurnHeading = heading;

        double degreesToGo = Math.abs(Math.toDegrees(PollenOdometryDrive.wrap(heading - step.targetHeading)));
        double powerLimit = PollenOdometryDrive.rampedTurnPower(elapsed, step.maxPower);
        double turn = PollenOdometryDrive.turnPower(step.targetHeading, powerLimit, heading, turnRate);

        // 1. Arrived? Only when inside the tolerance AND no longer turning.
        boolean insideTolerance = degreesToGo <= PollenOdometryDrive.HEADING_TOLERANCE_DEG;
        boolean stopped = haveRate
                && Math.abs(Math.toDegrees(turnRate)) <= PollenOdometryDrive.SETTLED_DEG_PER_SECOND;
        if (insideTolerance && stopped) return advance();

        // 2. Out of time? Almost there: hand over to the next step. Farther away: stop the route.
        if (elapsed >= step.timeoutSeconds) {
            if (degreesToGo <= PollenOdometryDrive.TURN_NEAR_ENOUGH_DEG) {
                warning = String.format(Locale.US, "%s ended %.1f deg short after %.1f s (close enough)",
                        step.name, degreesToGo, elapsed);
                return advance();
            }
            warning = String.format(Locale.US, "%s ran out of time %.1f deg short after %.1f s",
                    step.name, degreesToGo, elapsed);
            return stop(step.name + " TIMED OUT - STOPPED");
        }

        // 3. Stuck? Wheels driven, not almost there, and the heading has hardly changed.
        boolean almostThere = degreesToGo <= PollenOdometryDrive.TURN_NEAR_ENOUGH_DEG;
        boolean firstLook = Double.isNaN(progressHeading);
        boolean madeProgress = !firstLook && Math.abs(Math.toDegrees(
                PollenOdometryDrive.wrap(heading - progressHeading))) >= PollenOdometryDrive.NO_TURN_PROGRESS_MIN_DEG;
        if (turn == 0 || almostThere || firstLook || madeProgress) {
            progressHeading = heading;
            progressAt = now;
        } else if (now - progressAt >= PollenOdometryDrive.NO_PROGRESS_SECONDS) {
            warning = String.format(Locale.US,
                    "%s: wheels driven but heading changed < %.0f deg in %.2f s, %.1f deg short",
                    step.name, PollenOdometryDrive.NO_TURN_PROGRESS_MIN_DEG, now - progressAt, degreesToGo);
            return stop(step.name + " NOT TURNING - STOPPED");
        }

        // 4. Still turning: pass the controller's power on.
        Output out = new Output(step.name);
        out.turn = turn;
        out.degreesToGo = degreesToGo;
        return out;
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
     * any would be over 1. Output order: frontLeft, frontRight, backLeft, backRight. An Output's
     * forward and strafe are in ROUTE directions (as if facing the hive); the autonomous OpModes
     * use PollenFacing.wheelPowers, which turns them into the robot's own directions first. */
    public static double[] wheelPowers(Output out) {
        return wheelPowers(out.forward, out.strafe, out.turn);
    }

    /** The same mixer for powers in the robot's own directions (forward +, strafe + = right,
     * turn + = clockwise). PollenFacing uses it to drive a robot that starts with its back to the hive. */
    public static double[] wheelPowers(double forward, double strafe, double turn) {
        double[] powers = {forward + strafe + turn,
                forward - strafe - turn,
                forward - strafe + turn,
                forward + strafe - turn};
        double max = 1;
        for (double power : powers) max = Math.max(max, Math.abs(power));
        for (int i = 0; i < powers.length; i++) powers[i] /= max;
        return powers;
    }
}
