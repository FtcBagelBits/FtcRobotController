package org.firstinspires.ftc.teamcode.pollen;

import java.util.Locale;

/**
 * Records what the driver and the robot did in PollenBot TeleOp, so a practice run can be studied
 * afterward and a rehearsed move can be turned into part of an autonomous route.
 *
 * One CSV file per TeleOp run, in FIRST/pollen-hive/teleop on the Control Hub. A row is written
 * 20 times a second while anything is happening and once a second while the robot sits idle.
 * Each row has the sticks and buttons, the power sent to every motor, the launcher's measured
 * speed, the battery, and (if the robot has its Pinpoint odometry) where the robot is.
 *
 * REHEARSALS: the driver marks the part of the run that should become autonomous.
 *   Circle = this is where the sequence STARTS (like a record button).
 *   Triangle = the sequence is OVER (like a stop button).
 * Put the robot where the autonomous would begin that move, press Circle, drive the move, press
 * Triangle. On the laptop, tools/teleop-rehearsal.py turns each marked part into waypoints measured
 * from the spot where Circle was pressed. Marking changes nothing about how the robot drives.
 *
 * Pure Java (no SDK), so it is tested on a computer. Recording must never stop the robot: every
 * method catches its own errors and simply stops recording.
 */
public final class PollenTeleOpRecorder {
    /** While the robot is doing something: one row this often (20 a second). */
    public static final double ACTIVE_SECONDS = 0.05;
    /** While nothing is commanded and no rehearsal is marked: one row this often. */
    public static final double IDLE_SECONDS = 1.0;

    /** What happened in one loop. The OpMode fills this in; nothing here touches hardware. */
    public static final class Sample {
        /** Driver inputs: sticks -1..1 (forward, right, clockwise are positive), triggers 0..1. */
        public double stickForward, stickStrafe, stickTurn, triggerIn, triggerOut;
        public boolean r1, l1;
        /** Power sent to frontLeft, frontRight, backLeft, backRight. */
        public final double[] wheels = new double[4];
        public double intake, agitator;
        /** Launcher speed asked for and measured, in encoder ticks per second. */
        public double launcherTarget, launcherMeasured = Double.NaN;
        public boolean launching, feeding, reversing;
        /** Pinpoint status text, or why there is no position. */
        public String odometry = "NO PINPOINT";
        /** Position since INIT in route directions (PollenFacing): inches toward the hive, inches
         * left, degrees counterclockwise. NaN without odometry. */
        public double x = Double.NaN, y = Double.NaN, headingDegrees = Double.NaN;
        /** The same position exactly as the Pinpoint reports it (robot's own directions, radians). */
        public double rawX = Double.NaN, rawY = Double.NaN, rawHeading = Double.NaN;
        public double batteryVolts = Double.NaN;
    }

    /** What a button press just did, so the OpMode can buzz the gamepad. */
    public enum Mark { NONE, STARTED, ENDED }

    private final PollenHiveTrace file;
    private double lastRowAt = Double.NEGATIVE_INFINITY, lastLoopAt = Double.NaN, slowestLoop;
    private double rehearsalStartedAt;
    private int rehearsal, rehearsals;
    private boolean startWasDown, endWasDown, stopped;

    /** @param file where rows go, or null when recording could not start (TeleOp still runs). */
    public PollenTeleOpRecorder(PollenHiveTrace file) { this.file = file; }

    /** START pressed: the column names, then one row with every setting for this run. */
    public void start(long wallMillis, String settings, Sample s) {
        if (file == null) return;
        try {
            file.row(0, false, "t_s", "wall_ms", "kind", "rehearsal", "note",
                    "stick_forward", "stick_strafe", "stick_turn", "trigger_in", "trigger_out", "r1", "l1",
                    "wheel_fl", "wheel_fr", "wheel_bl", "wheel_br", "intake_cmd", "agitator_cmd",
                    "launcher_target_tps", "launcher_measured_tps", "launching", "feeding", "reversing",
                    "odom_status", "x_in", "y_in", "heading_deg", "raw_x_in", "raw_y_in", "raw_heading_rad",
                    "battery_v", "slowest_loop_ms");
            row(0, wallMillis, "START", settings, s, true);
        } catch (RuntimeException ignored) { }
    }

    /**
     * Every loop after START.
     * @param markStart Circle is down right now; @param markEnd Triangle is down right now.
     * @return what the buttons just did (only on the loop where the button goes down).
     */
    public Mark loop(double now, long wallMillis, boolean markStart, boolean markEnd, Sample s) {
        Mark mark = Mark.NONE;
        try {
            if (!Double.isNaN(lastLoopAt)) slowestLoop = Math.max(slowestLoop, now - lastLoopAt);
            lastLoopAt = now;
            // Buttons act once, when they go down. Circle while already rehearsing ends that
            // rehearsal and starts the next one at the same spot.
            if (markStart && !startWasDown) {
                if (rehearsal != 0) row(now, wallMillis, "MARK_END", "ended by the next Circle", s, true);
                rehearsal = ++rehearsals;
                rehearsalStartedAt = now;
                row(now, wallMillis, "MARK_START", "", s, true);
                mark = Mark.STARTED;
            } else if (markEnd && !endWasDown && rehearsal != 0) {
                row(now, wallMillis, "MARK_END", "", s, true);
                rehearsal = 0;
                mark = Mark.ENDED;
            }
            startWasDown = markStart;
            endWasDown = markEnd;
            double every = rehearsal != 0 || active(s) ? ACTIVE_SECONDS : IDLE_SECONDS;
            // The tiny allowance keeps 0.05 s from being missed by a rounding error in the clock.
            if (mark == Mark.NONE && now - lastRowAt >= every - 1e-6) row(now, wallMillis, "SAMPLE", "", s, false);
        } catch (RuntimeException ignored) { }
        return mark;
    }

    /** The OpMode ended (STOP pressed, or the match timer). A rehearsal still open ends here. */
    public void stop(double now, long wallMillis, Sample s) {
        if (stopped) return;
        stopped = true;
        try {
            if (rehearsal != 0) row(now, wallMillis, "MARK_END", "ended by STOP", s, true);
            rehearsal = 0;
            row(now, wallMillis, "STOP", "", s, true);
        } catch (RuntimeException ignored) { }
    }

    /** One line for the Driver Hub screen. */
    public String screen(double now) {
        String recording = file == null ? "NOT RECORDING" : "open".equals(file.status)
                ? String.format(Locale.US, "recording (%d kB)", file.bytes() / 1024) : "STOPPED: " + file.status;
        if (rehearsal != 0) return String.format(Locale.US, "REHEARSAL %d: %.1f s (Triangle = end) | %s",
                rehearsal, now - rehearsalStartedAt, recording);
        return recording + " | Circle = start a rehearsal" + (rehearsals == 0 ? "" : " (" + rehearsals + " so far)");
    }

    /** The rehearsal being recorded right now (1, 2, 3...), or 0. */
    public int rehearsal() { return rehearsal; }
    /** How many rehearsals have been started in this run. */
    public int rehearsals() { return rehearsals; }

    /** Is the driver asking for anything, or is anything still spinning? */
    private static boolean active(Sample s) {
        boolean active = s.r1 || s.l1 || s.launching || s.feeding || s.reversing
                || s.intake != 0 || s.agitator != 0 || s.launcherTarget != 0
                || Math.abs(s.launcherMeasured) > 50; // still coasting down after a shot
        for (double w : s.wheels) active |= w != 0;
        return active;
    }

    private void row(double now, long wallMillis, String kind, String note, Sample s, boolean flush) {
        if (file == null) return;
        lastRowAt = now;
        file.row(now, flush, now, wallMillis, kind, rehearsal, note,
                s.stickForward, s.stickStrafe, s.stickTurn, s.triggerIn, s.triggerOut, s.r1, s.l1,
                s.wheels[0], s.wheels[1], s.wheels[2], s.wheels[3], s.intake, s.agitator,
                s.launcherTarget, s.launcherMeasured, s.launching, s.feeding, s.reversing,
                s.odometry, s.x, s.y, s.headingDegrees, s.rawX, s.rawY, s.rawHeading,
                s.batteryVolts, slowestLoop * 1000);
        slowestLoop = 0;
    }
}
