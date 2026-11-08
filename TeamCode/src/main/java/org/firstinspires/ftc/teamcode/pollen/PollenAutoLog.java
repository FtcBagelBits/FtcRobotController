package org.firstinspires.ftc.teamcode.pollen;

import java.util.Locale;

/**
 * Writes what an autonomous run did to the robot log, so a strange match can be explained
 * afterward ("why did it stop?", "which route was selected?"). The OpMode sends each line to the
 * SDK's RobotLog, which puts it in robotControllerLog.txt and in the match log. Pull those files
 * after every event (see CLAUDE.md, "After an event: pull the robot logs").
 *
 * What gets logged: the route line during INIT whenever it changes (for example someone pressed
 * Circle), every setting at START, each phase change, warnings, the final result, the Pinpoint
 * status when it changes, and a short sample of position, wheel powers and battery every
 * SAMPLE_SECONDS while anything is moving.
 *
 * Pure Java (no SDK), so it is tested on a computer. Logging must never stop the robot: every
 * method catches its own errors and simply skips the line.
 */
public final class PollenAutoLog {
    /** Tag to search for in robotControllerLog.txt. */
    public static final String TAG = "PollenAuto";
    /** While anything is moving, one position/power line this often. */
    public static final double SAMPLE_SECONDS = 0.25;

    /** Where lines go. On the robot: RobotLog. In tests: a list. */
    public interface Sink { void line(String text); }
    /** Battery voltage, read only when a line needs it (each read talks to the hub). */
    public interface Battery { double volts(); }

    private final String mode;
    private final Sink sink;
    private String lastRoute, lastPhase, lastWarning = "", lastStatus;
    private double lastSampleAt = Double.NEGATIVE_INFINITY;
    private boolean resultLogged;

    public PollenAutoLog(String mode, Sink sink) {
        this.mode = mode;
        this.sink = sink;
    }

    /** INIT: log the route whenever it changes. */
    public void initRoute(String route) {
        try {
            if (route == null || route.equals(lastRoute)) return;
            lastRoute = route;
            emit("INIT " + mode + " route=" + route);
        } catch (RuntimeException ignored) { }
    }

    /** Log the Pinpoint status whenever it changes (INIT or running). */
    public void status(String status) {
        try {
            if (status == null || status.equals(lastStatus)) return;
            lastStatus = status;
            emit("PINPOINT status=" + status);
        } catch (RuntimeException ignored) { }
    }

    /** START pressed: build, mode and every setting, so the run can be reproduced. */
    public void start(String settings, String hardware, Battery battery) {
        try {
            emit("START " + mode + " build=" + PollenConfig.BUILD + " " + settings + " " + hardware
                    + " battery=" + volts(battery));
        } catch (RuntimeException ignored) { }
    }

    /** START refused (hardware missing, over the time budget...). */
    public void notStarted(String why) {
        try { emit("NOT STARTED: " + why); } catch (RuntimeException ignored) { }
    }

    /**
     * Every loop after START. Pose is inches/radians from the last zero (NaN in timed modes);
     * wheels are FL, FR, BL, BR; launcherMeasured is the encoder reading in ticks/s.
     */
    public void loop(double t, PollenSequence.Output out, String warning, double x, double y,
                     double heading, double[] wheels, double launcherMeasured, Battery battery) {
        try {
            if (warning != null && !warning.isEmpty() && !warning.equals(lastWarning)) {
                lastWarning = warning;
                emit(String.format(Locale.US, "t=%.2f WARNING %s", t, warning));
            }
            if (!out.phase.equals(lastPhase) && !out.phase.startsWith("NEXT: ")) {
                lastPhase = out.phase;
                if (out.finished) {
                    if (!resultLogged) {
                        resultLogged = true;
                        emit(String.format(Locale.US, "t=%.2f RESULT %s | %s battery=%s",
                                t, out.phase, pose(x, y, heading), volts(battery)));
                    }
                } else {
                    emit(String.format(Locale.US, "t=%.2f PHASE %s | %s battery=%s",
                            t, out.phase, pose(x, y, heading), volts(battery)));
                }
            }
            boolean moving = out.launcherVelocity != 0 || out.feederPower != 0;
            for (double w : wheels) moving |= w != 0;
            if (moving && !out.finished && t - lastSampleAt >= SAMPLE_SECONDS) {
                lastSampleAt = t;
                emit(String.format(Locale.US,
                        "t=%.2f %s togo=%.1f wheels=%.2f,%.2f,%.2f,%.2f launcher=%.0f/%.0f feed=%.2f battery=%s",
                        t, pose(x, y, heading), out.inchesToGo, wheels[0], wheels[1], wheels[2], wheels[3],
                        out.launcherVelocity, launcherMeasured, out.feederPower, volts(battery)));
            }
        } catch (RuntimeException ignored) { }
    }

    /** The OpMode ended (end of autonomous, STOP pressed, or an error). */
    public void stop(double t) {
        try {
            emit(String.format(Locale.US, "t=%.2f STOP (OpMode ended; last phase: %s)", t, lastPhase));
        } catch (RuntimeException ignored) { }
    }

    private static String pose(double x, double y, double heading) {
        return String.format(Locale.US, "x=%.1f y=%.1f h=%.1f", x, y, Math.toDegrees(heading));
    }

    private static String volts(Battery battery) {
        try {
            return battery == null ? "?" : String.format(Locale.US, "%.2fV", battery.volts());
        } catch (RuntimeException e) {
            return "?";
        }
    }

    private void emit(String text) {
        try { sink.line(text); } catch (RuntimeException ignored) { }
    }
}
