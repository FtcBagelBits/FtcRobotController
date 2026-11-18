package org.firstinspires.ftc.teamcode.pollen;

import java.util.ArrayList;
import java.util.List;

/** Collection from an established entry pose. Inputs are the same route inches/radians
 * as PollenHiveAuto. Call once per loop; a caller can enter at any position/heading
 * without resetting its localizer. Positive distances in the plans point out the intake. */
public final class PollenCollectionSequence {
    public enum Kind { GARDEN, FLOWER }

    private static PollenHiveSettings.Row row(String key, String label, double initial,
                                             double step, double min, double max) {
        return new PollenHiveSettings.Row(key, label, initial, initial, step, min, max);
    }

    private static final PollenHiveSettings.Row[] GARDEN = {
        row("gap", "Starting front edge to wall cm", 15.24, 1, 3, 100),
        row("clear", "Final front edge to wall cm", 2.54, .5, 0, 30),
        row("inPower", "Drive in power", .20, .05, .10, .60),
        row("passes", "Total inward passes", 3, 1, 1, 8),
        row("back", "Backoff between passes cm", 10, 1, 2, 60),
        row("outPower", "Backoff / lane shift power", .25, .05, .10, .60),
        row("shift", "Lane step per retry cm (+ robot left)", 0, .5, -15, 15),
        row("hold", "Intake dwell each pass s", .5, .1, 0, 3),
        row("intake", "Intake power", .5, .05, .05, 1),
        row("exit", "Back out after last pass cm", 12.7, 1, 0, 60),
    };
    private static final PollenHiveSettings.Row[] FLOWER = {
        row("gap", "Start to fully inserted mark cm", 20, 1, 4, 100),
        row("creep", "Last cm by timed push", 3, .5, 0, 10),
        row("inPower", "Approach power", .20, .05, .10, .60),
        row("lower", "Let drainer lower s", .4, .1, .1, 2),
        row("down", "Drainer down position", PollenTeleOpTuning.DEFAULT_DRAINER_DOWN, .01, 0, 1),
        row("seatPower", "Final push power", .15, .01, .05, .35),
        row("seatTime", "Final push s", .3, .05, 0, 1),
        row("shakes", "Left-right cycles", 3, 1, 0, 8),
        row("amplitude", "Travel each side of center cm", 3, .5, 2, 12),
        row("sidePower", "Sideways power", .20, .05, .10, .50),
        row("back", "Full withdrawal cm", 18, 1, 4, 60),
        row("stroke", "Cleanup forward stroke cm", 7, 1, 2, 40),
        row("repeats", "Cleanup forward/back cycles", 2, 1, 0, 8),
        row("outPower", "Withdrawal / cleanup power", .25, .05, .10, .60),
        row("hold", "Final intake dwell s", .5, .1, 0, 3),
    };
    public static PollenHiveSettings.Values settings(Kind kind) {
        return new PollenHiveSettings.Values(kind == Kind.FLOWER ? FLOWER : GARDEN, false);
    }

    public static void validate(Kind kind, PollenHiveSettings.Values s) {
        if (s.rows != (kind == Kind.FLOWER ? FLOWER : GARDEN))
            throw new IllegalArgumentException("Wrong collection profile");
        for (int i = 0; i < s.rows.length; i++) {
            PollenHiveSettings.Row r = s.rows[i];
            double v = s.v[i];
            if (!Double.isFinite(v) || v < r.min || v > r.max)
                throw new IllegalArgumentException("Out of range: " + r.label);
        }
        if (kind == Kind.GARDEN) {
            count(s, "passes");
            if (s.get("clear") >= s.get("gap"))
                throw new IllegalArgumentException("Start gap must exceed final clearance");
        } else {
            count(s, "shakes");
            count(s, "repeats");
            if (s.get("creep") >= s.get("gap") || (s.get("creep") > 0 && s.get("seatTime") == 0))
                throw new IllegalArgumentException("Timed push needs travel room and time");
            if (s.get("repeats") > 0 && s.get("stroke") >= s.get("back"))
                throw new IllegalArgumentException("Cleanup stroke must stay short of full insertion");
            if (s.get("down") >= PollenTeleOpTuning.DEFAULT_DRAINER_UP)
                throw new IllegalArgumentException("Down position must be below the UP setting");
        }
    }

    private static int count(PollenHiveSettings.Values s, String key) {
        double n = s.get(key);
        if (n != Math.rint(n)) throw new IllegalArgumentException(key + " requires a whole number");
        return (int)n;
    }

    public static final class Output {
        public double forward, strafe, turn, intake;
        public boolean drainerDown, done;
        public String phase, fault;
    }
    private enum Stage { GARDEN, APPROACH, SEAT, SHAKE, CLEANUP, DONE, FAULT }
    private final Kind kind;
    private final PollenHiveSettings.Values s;
    private Stage stage;
    private PollenHiveMotion motion;
    private final double heading;
    private double anchorX, anchorY, seatStart, previous = Double.NaN;
    private String fault;

    public PollenCollectionSequence(Kind kind, PollenHiveSettings.Values settings,
                                    double entryX, double entryY, double entryHeading) {
        validate(kind, settings);
        if (!Double.isFinite(entryX) || !Double.isFinite(entryY) || !Double.isFinite(entryHeading))
            throw new IllegalArgumentException("Invalid collection entry pose");
        this.kind = kind;
        s = settings(kind);
        System.arraycopy(settings.v, 0, s.v, 0, s.v.length);
        anchorX = entryX;
        anchorY = entryY;
        heading = entryHeading;
        stage = kind == Kind.GARDEN ? Stage.GARDEN : Stage.APPROACH;
        motion = route(stage);
    }

    // Offsets are cm forward/left in the physical robot's orientation at entry.
    private PollenHiveRoute.Node node(String name, double forward, double left, double power, double dwell) {
        double[] offset = PollenFacing.routePose(forward / 2.54, left / 2.54, 0);
        double c = Math.cos(heading), sin = Math.sin(heading);
        return new PollenHiveRoute.Node(name, anchorX + c * offset[0] - sin * offset[1],
                anchorY + sin * offset[0] + c * offset[1], Math.toDegrees(heading), power, false, true, dwell);
    }

    private PollenHiveMotion route(Stage part) {
        List<PollenHiveRoute.Node> nodes = new ArrayList<>();
        double back = s.get("back"), out = s.get("outPower");
        if (part == Stage.GARDEN) {
            double near = s.get("gap") - s.get("clear");
            for (int i = 0; i < count(s, "passes"); i++) {
                double lane = i * s.get("shift");
                if (i > 0 && lane != 0) nodes.add(node("Shift to lane " + (i + 1), near - back, lane, out, 0));
                nodes.add(node("Inward pass " + (i + 1), near, lane, s.get("inPower"), 0));
                if (s.get("hold") > 0) nodes.add(node("Ingest " + (i + 1), near, lane, 0, s.get("hold")));
                boolean last = i + 1 == count(s, "passes");
                double retreat = last ? s.get("exit") : back;
                if (retreat > 0) nodes.add(node(last ? "Exit garden" : "Back off " + (i + 1), near - retreat, lane, out, 0));
            }
        } else if (part == Stage.APPROACH) {
            nodes.add(node("Lower drainer", 0, 0, 0, s.get("lower")));
            nodes.add(node("Approach flower", s.get("gap") - s.get("creep"), 0, s.get("inPower"), 0));
        } else if (part == Stage.SHAKE) {
            for (int i = 1; i <= count(s, "shakes"); i++) {
                nodes.add(node("Left " + i, 0, s.get("amplitude"), s.get("sidePower"), 0));
                nodes.add(node("Right " + i, 0, -s.get("amplitude"), s.get("sidePower"), 0));
            }
            if (count(s, "shakes") > 0) nodes.add(node("Center drainer", 0, 0, s.get("sidePower"), 0));
            nodes.add(node("Withdraw drainer", -back, 0, out, 0));
        } else {
            for (int i = 1; i <= count(s, "repeats"); i++) {
                nodes.add(node("Ingest forward " + i, -back + s.get("stroke"), 0, out, 0));
                nodes.add(node("Back out " + i, -back, 0, out, 0));
            }
            if (s.get("hold") > 0) nodes.add(node("Finish ingesting", -back, 0, 0, s.get("hold")));
        }
        return new PollenHiveMotion(nodes.toArray(new PollenHiveRoute.Node[0]));
    }

    /** Time is monotonic seconds. A fault, STOP or DONE latches zero drive/intake and drainer up. */
    public Output tick(double now, double x, double y, double h, boolean odometryReady) {
        if (stage == Stage.DONE || stage == Stage.FAULT) return status();
        if (!odometryReady || !Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(h))
            return fail("Pinpoint not ready or invalid pose");
        if (!Double.isFinite(now) || now < previous || now - previous > .25) return fail("Motion loop stalled");
        previous = now;
        if (stage == Stage.SEAT) {
            if (now - seatStart < s.get("seatTime")) {
                Output o = status();
                o.phase = "Seat drainer";
                o.forward = PollenFacing.routeForwardFromRobot(s.get("seatPower"));
                return o;
            }
            // A wall is a hard stop. Shake around the pose actually reached, not a target inside it.
            anchorX = x;
            anchorY = y;
            stage = Stage.SHAKE;
            motion = route(stage);
        }
        PollenHiveMotion.Output m = motion.tick(now, x, y, h);
        if (m.fault != null) return fail(m.fault);
        if (m.done) {
            switch (stage) {
                case APPROACH: stage = Stage.SEAT; seatStart = now; break;
                case SHAKE: stage = Stage.CLEANUP; motion = route(stage); break;
                default: stage = Stage.DONE; break;
            }
            return status();
        }
        Output o = status();
        o.phase = m.phase;
        o.forward = m.forward;
        o.strafe = m.strafe;
        o.turn = m.turn;
        return o;
    }

    public Output stop() {
        if (stage != Stage.FAULT) stage = Stage.DONE;
        return status();
    }

    private Output fail(String reason) {
        fault = reason;
        stage = Stage.FAULT;
        return status();
    }

    private Output status() {
        Output o = new Output();
        o.phase = stage.name();
        o.done = stage == Stage.DONE;
        o.fault = fault;
        if (!o.done && fault == null) {
            o.intake = kind == Kind.FLOWER ? 1 : s.get("intake");
            o.drainerDown = kind == Kind.FLOWER && stage != Stage.CLEANUP;
        }
        return o;
    }
}
