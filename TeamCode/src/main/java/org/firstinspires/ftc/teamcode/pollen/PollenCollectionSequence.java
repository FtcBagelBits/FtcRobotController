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
        row("repeats", "Backoff and re-entry repeats", 1, 1, 0, 8),
        row("backoffIn", "Backoff distance in", 3, .25, 1, 24),
        row("sidePower", "Side wall approach power", .30, .05, .10, .60),
        row("inPower", "Forward wall approach power", .25, .05, .10, .60),
        row("outPower", "Backoff power", .25, .05, .10, .60),
        row("intake", "Intake power", 1, .05, .05, 1),
    };
    private static final PollenHiveSettings.Row[] FLOWER = {
        row("gap", "Start to fully inserted mark cm", 24, 1, 4, 100),
        row("creep", "Switch to seating power this many cm short", 3, .5, 0, 10),
        // 2026-11-19 secondary trace: the flower took 8.7 s. Faster approach and withdrawal (v35).
        row("inPower", "Approach power", .30, .05, .10, .60),
        row("lower", "Let drainer lower s", .4, .1, .1, 2),
        row("down", "Drainer down position", PollenTeleOpTuning.DEFAULT_DRAINER_DOWN, .01, 0, 1),
        row("seatPower", "Final wall approach power", .15, .01, .05, .35),
        row("shakes", "Left-right cycles", 3, 1, 0, 8),
        row("amplitude", "Travel each side of center cm", 3.5, .5, 2, 12),
        // Own key since v35, so the old saved 0.40-0.50 "sidePower" no longer slows the swings:
        // the drive team asked for at least 50 % more swing speed to empty the flower (0.50 -> 0.75).
        row("shakePower", "Swing sideways power", .75, .05, .10, .90),
        row("back", "Full withdrawal cm", 18, 1, 4, 60),
        row("stroke", "Cleanup forward stroke cm", 7, 1, 2, 40),
        row("repeats", "Cleanup forward/back cycles", 2, 1, 0, 8),
        row("outPower", "Withdrawal / cleanup power", .40, .05, .10, .60),
        row("hold", "Final intake dwell s", 1, .1, 0, 3),
        row("intake", "Intake power", 1, .05, .05, 1),
    };
    static PollenHiveSettings.Row[] rows(Kind kind) { return kind == Kind.FLOWER ? FLOWER : GARDEN; }

    public static PollenHiveSettings.Values settings(Kind kind) {
        return new PollenHiveSettings.Values(rows(kind), false);
    }

    public static void validate(Kind kind, PollenHiveSettings.Values s) {
        if (s.rows != rows(kind))
            throw new IllegalArgumentException("Wrong collection profile");
        for (int i = 0; i < s.rows.length; i++) {
            PollenHiveSettings.Row r = s.rows[i];
            double v = s.v[i];
            if (!Double.isFinite(v) || v < r.min || v > r.max)
                throw new IllegalArgumentException("Out of range: " + r.label);
        }
        if (kind == Kind.GARDEN) {
            count(s, "repeats");
        } else {
            count(s, "shakes");
            count(s, "repeats");
            if (s.get("creep") >= s.get("gap"))
                throw new IllegalArgumentException("Seating distance must be shorter than the start gap");
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
        public String phase, fault, contactLimit;
    }
    private enum Stage { GARDEN_SIDE, GARDEN_IN, GARDEN_OUT, APPROACH, SEAT, SHAKE, WITHDRAW, CLEANUP, DONE, FAULT }
    private final Kind kind;
    private final PollenHiveSettings.Values s;
    private Stage stage;
    private PollenHiveMotion motion;
    private final double heading;
    private double anchorX, anchorY, previous = Double.NaN;
    private int pass = 1;
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
        stage = kind == Kind.GARDEN ? Stage.GARDEN_SIDE : Stage.APPROACH;
        motion = route(stage);
    }

    // Offsets are cm forward/left in the physical robot's orientation at entry.
    private PollenHiveRoute.Node node(String name, double forward, double left, double power, double dwell) {
        double[] offset = routeVector(forward / 2.54, left / 2.54);
        return new PollenHiveRoute.Node(name, anchorX + offset[0], anchorY + offset[1],
                Math.toDegrees(heading), power, false, true, dwell);
    }

    /** A flower swing leg: same target as node(), braking later so the swing power is reached. */
    private PollenHiveRoute.Node swing(String name, double left, double power) {
        double[] offset = routeVector(0, left / 2.54);
        return new PollenHiveRoute.Node(name, anchorX + offset[0], anchorY + offset[1],
                Math.toDegrees(heading), power, false, true, 0, true);
    }

    private double[] routeVector(double forward, double left) {
        double[] offset = PollenFacing.routePose(forward, left, 0);
        double c = Math.cos(heading), sin = Math.sin(heading);
        return new double[]{c * offset[0] - sin * offset[1], sin * offset[0] + c * offset[1]};
    }

    private PollenHiveRoute.Node wallDrive(String name, double forward, double left, double power) {
        double[] direction = routeVector(forward, left);
        return new PollenHiveRoute.Node(name, direction[0], direction[1],
                Math.toDegrees(heading), power, false, true, 0);
    }

    private PollenHiveMotion route(Stage part) {
        List<PollenHiveRoute.Node> nodes = new ArrayList<>();
        double out = s.get("outPower");
        PollenHiveMotion.Contact contact = PollenHiveMotion.Contact.STRICT;
        if (part == Stage.GARDEN_SIDE || part == Stage.GARDEN_IN) {
            contact = PollenHiveMotion.Contact.DRIVE_UNTIL_BLOCKED;
            if (part == Stage.GARDEN_SIDE) {
                nodes.add(wallDrive("Find right wall", 0, -1, s.get("sidePower")));
            } else {
                nodes.add(wallDrive("Inward pass " + pass, 1, 0, s.get("inPower")));
            }
        } else if (part == Stage.GARDEN_OUT) {
            nodes.add(node("Back off " + pass, -s.get("backoffIn") * 2.54, 0, out, 0));
        } else if (part == Stage.APPROACH) {
            contact = PollenHiveMotion.Contact.ALLOWED;
            nodes.add(node("Lower drainer", 0, 0, 0, s.get("lower")));
            nodes.add(node("Approach flower", s.get("gap") - s.get("creep"), 0, s.get("inPower"), 0));
        } else if (part == Stage.SEAT) {
            contact = PollenHiveMotion.Contact.DRIVE_UNTIL_BLOCKED;
            nodes.add(wallDrive("Seat drainer", 1, 0, s.get("seatPower")));
        } else if (part == Stage.SHAKE) {
            contact = PollenHiveMotion.Contact.ALLOWED;
            for (int i = 1; i <= count(s, "shakes"); i++) {
                nodes.add(swing("Left " + i, s.get("amplitude"), s.get("shakePower")));
                nodes.add(swing("Right " + i, -s.get("amplitude"), s.get("shakePower")));
            }
            if (count(s, "shakes") > 0) nodes.add(swing("Center drainer", 0, s.get("shakePower")));
        } else if (part == Stage.WITHDRAW) {
            nodes.add(node("Withdraw drainer", -s.get("back"), 0, out, 0));
        } else {
            double back = s.get("back");
            for (int i = 1; i <= count(s, "repeats"); i++) {
                nodes.add(node("Ingest forward " + i, -back + s.get("stroke"), 0, out, 0));
                nodes.add(node("Back out " + i, -back, 0, out, 0));
            }
            if (s.get("hold") > 0) nodes.add(node("Finish ingesting", -back, 0, 0, s.get("hold")));
        }
        return new PollenHiveMotion(nodes.toArray(new PollenHiveRoute.Node[0]), contact);
    }

    private void startMove(Stage next, double x, double y) {
        anchorX = x;
        anchorY = y;
        stage = next;
        motion = route(stage);
    }

    /** Time is monotonic seconds. A fault, STOP or DONE latches zero drive/intake and drainer up. */
    public Output tick(double now, double x, double y, double h, boolean odometryReady) {
        if (stage == Stage.DONE || stage == Stage.FAULT) return status();
        if (!odometryReady || !Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(h))
            return fail("Pinpoint not ready or invalid pose");
        if (!Double.isFinite(now) || now < previous || now - previous > .25) return fail("Motion loop stalled");
        previous = now;
        PollenHiveMotion.Output m = motion.tick(now, x, y, h);
        if (m.fault != null) return fail(m.detail.isEmpty() ? m.fault : m.fault + " | " + m.detail);
        if ((stage == Stage.APPROACH || stage == Stage.SEAT) && m.contactLimit) startMove(Stage.SHAKE, x, y);
        if (m.done) {
            switch (stage) {
                case GARDEN_SIDE: startMove(Stage.GARDEN_IN, x, y); break;
                case GARDEN_IN:
                    if (pass > count(s, "repeats")) stage = Stage.DONE;
                    else startMove(Stage.GARDEN_OUT, x, y);
                    break;
                case GARDEN_OUT:
                    pass++;
                    startMove(Stage.GARDEN_IN, x, y);
                    break;
                case APPROACH: startMove(Stage.SEAT, x, y); break;
                case SHAKE:
                    // A constrained drainer may still be off center. Pull straight out from there.
                    startMove(Stage.WITHDRAW, x, y);
                    break;
                case WITHDRAW: stage = Stage.CLEANUP; motion = route(stage); break;
                default: stage = Stage.DONE; break;
            }
            return status();
        }
        Output o = status();
        o.phase = m.phase;
        o.forward = m.forward;
        o.strafe = m.strafe;
        o.turn = m.turn;
        if (m.contactLimit) o.contactLimit = m.detail;
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
            o.intake = s.get("intake");
            o.drainerDown = kind == Kind.FLOWER;
        }
        return o;
    }
}
