package org.firstinspires.ftc.teamcode.pollen;

import java.util.Locale;
import java.util.Properties;

/** Every number the experimental hive modes use, with its DEFAULT. One place, pure Java.
 *
 * How the team iterates:
 *  1. Defaults here should be right: place the robot, pick the auto, INIT, START.
 *  2. To try something, change a row in the INIT menu. Rows that differ from their default are
 *     marked CHANGED. Square saves the changed rows on the Control Hub (FIRST/pollen-hive/*.properties)
 *     so they stay until reset. Only differences are saved, never a full copy.
 *  3. When a change is proven, make it the new default: pull the folder and run
 *     tools/hive-promote-defaults.py, which rewrites the numbers in the row() lines below. Commit,
 *     install: the row is no longer CHANGED.
 *
 * Each row: key (file name of the setting; the promote tool finds it), label, default for PRIMARY,
 * default for SECONDARY, step for one D-pad press, min, max. CAMERA rows use the same default for both.
 * Route distances are cm from START, forward = toward the hive, left = left looking at the hive;
 * derivations are in docs/POLLEN_HIVE_TESTING.md (CAD, docs/POLLEN_FIELD_LAYOUT.md).
 */
public final class PollenHiveSettings {
    private PollenHiveSettings() { }

    public static final class Row {
        public final String key, label;
        public final double primary, secondary, step, min, max;
        Row(String key, String label, double primary, double secondary, double step, double min, double max) {
            this.key = key; this.label = label; this.primary = primary; this.secondary = secondary;
            this.step = step; this.min = min; this.max = max;
        }
    }
    private static Row row(String key, String label, double primary, double secondary, double step, double min, double max) {
        return new Row(key, label, primary, secondary, step, min, max);
    }

    /** Execution is never saved: every INIT starts at its default (LIVE), so a forgotten DRY test
     * can't make a match auto sit still. */
    public static final String NOT_SAVED = "execution";

    /** Auto rows. The index is the PollenHiveRoute.v[] index: do not reorder. */
    public static final Row[] ROUTE = {
        row("execution", "Execution", 2, 2, 1, 0, 2),
        row("side", "Side", 0, 0, 1, 0, 2),
        row("startDelay", "Start delay s", 0, 0, 0.5, 0, 20),
        row("firstFeed", "First feed s", 1.5, 1.5, 0.5, 0, 20),
        row("secondFeed", "Second feed s", 1.5, 1.5, 0.5, 0, 20),
        row("firstWindow", "First shot window s", 5, 5, 0.5, 0, 20),
        row("launcher", "Launcher ticks/s", 1180, 1180, 20, 500, 2500),
        row("travelPower", "Travel power", 0.6, 0.6, 0.05, 0.1, 0.8),
        row("turnPower", "Turn power", 0.4, 0.4, 0.05, 0.1, 0.8),
        row("collectPower", "Collection intake power", 0.5, 0.5, 0.05, 0, 1),
        row("collectHold", "Collection hold s", 0.5, 0.5, 0.5, 0, 20),
        row("feedIntake", "Feed intake power", 0.5, 0.5, 0.05, 0, 1),
        row("feedAgitator", "Feed agitator power", 1, 1, 0.05, 0, 1),
        row("clearForward", "Clearance forward cm", 15.24, 15.24, 1, -350, 350),
        row("firstTurnLeft", "First turn left cm", 43.8362319108, 43.8362319108, 1, -350, 350),
        row("gardenTurnLeft", "Garden turn left cm", 108.8221319108, 108.8221319108, 1, -350, 350),
        row("pickupLeft", "Pickup left cm", 121.5221319108, 121.5221319108, 1, -350, 350),
        row("pickupForward", "Pickup forward cm", 2.54, 2.54, 1, -350, 350),
        row("transitHeading", "Transit heading deg", -90, -90, 5, -180, 180),
        row("pickupHeading", "Pickup heading deg", 0, 0, 5, -180, 180),
        row("pickupPower", "Pickup drive power", 0.2, 0.2, 0.05, 0.1, 0.8),
        row("park", "Park", 1, 1, 1, 0, 1),
        row("park1Forward", "Park1 forward cm", 10, 20.32, 1, -350, 350),
        row("park1Left", "Park1 left cm", 0, 0, 1, -350, 350),
        row("park2Forward", "Park2 forward cm", 10, 20.32, 1, -350, 350),
        row("park2Left", "Park2 left cm", 63.5, 0, 1, -350, 350),
        row("park3Forward", "Park3 forward cm", 222.4659, 37.0459, 1, -350, 350),
        row("park3Left", "Park3 left cm", 109.8762319108, -116.76634, 1, -350, 350),
    };

    /** Camera rows, shared by all hive modes (one saved file). See IsHiveReadyForShot. */
    public static final Row[] CAMERA = {
        row("pipeline", "Limelight pipeline", 0, 0, 1, 0, 9),
        row("lensHeight", "Lens height in", 14.0, 14.0, 0.25, 2, 30),
        row("tilt", "Camera tilt up deg", Double.NaN, Double.NaN, 0.1, -10, 80),
        row("sideways", "Camera sideways correction in", 0.0, 0.0, 0.25, -12, 12),
        row("upHeight", "UP tag height in", 49.7, 49.7, 0.1, 30, 70),
        row("downHeight", "DOWN tag height in", 35.6, 35.6, 0.1, 20, 50),
        row("band", "UP/DOWN band +/- in", 3.0, 3.0, 0.25, 0.5, 6),
        row("oursMaxSideways", "Our cell max sideways in", 12.5, 12.5, 0.5, 2, 30),
    };

    /** Current values for one table (auto rows for primary or secondary, or the camera rows). */
    public static final class Values {
        public final Row[] rows;
        public final boolean secondary;
        public final double[] v;
        private final double[] saved;
        public Values(Row[] rows, boolean secondary) {
            this.rows = rows; this.secondary = secondary; v = new double[rows.length]; saved = new double[rows.length];
            resetAll(); System.arraycopy(v, 0, saved, 0, v.length);
        }
        public double def(int i) { return secondary ? rows[i].secondary : rows[i].primary; }
        public boolean changed(int i) { return !same(v[i], def(i)); }
        /** Edited in this INIT but not saved yet (Execution never counts: it is never saved). */
        public boolean unsaved(int i) { return !NOT_SAVED.equals(rows[i].key) && !same(v[i], saved[i]); }
        public int changedCount() { int n = 0; for (int i = 0; i < v.length; i++) if (changed(i)) n++; return n; }
        public int unsavedCount() { int n = 0; for (int i = 0; i < v.length; i++) if (unsaved(i)) n++; return n; }
        public void reset(int i) { v[i] = def(i); }
        public void resetAll() { for (int i = 0; i < v.length; i++) reset(i); }
        public void edit(int i, int delta) {
            Row r = rows[i];
            double x = Double.isNaN(v[i]) ? (r.key.equals("tilt") ? 40 : r.min) : v[i] + delta * r.step;
            x = Math.round(x * 1e6) / 1e6; // no 0.30000000000000004
            v[i] = Math.max(r.min, Math.min(r.max, x));
        }
        public int index(String key) {
            for (int i = 0; i < rows.length; i++) if (rows[i].key.equals(key)) return i;
            throw new IllegalArgumentException("No setting " + key);
        }
        public double get(String key) { return v[index(key)]; }
        /** The rows that differ from their defaults, for the Hub file. */
        public Properties overrides() {
            Properties p = new Properties();
            for (int i = 0; i < v.length; i++)
                if (changed(i) && !NOT_SAVED.equals(rows[i].key)) p.setProperty(rows[i].key, Double.toString(v[i]));
            return p;
        }
        /** Apply saved overrides on top of the defaults. Unknown keys and bad numbers are reported, not used. */
        public String load(Properties p) {
            StringBuilder skipped = new StringBuilder();
            for (String key : p.stringPropertyNames()) {
                if (key.startsWith("#") || NOT_SAVED.equals(key) || key.equals("build")) continue;
                try {
                    int i = index(key);
                    double x = Double.parseDouble(p.getProperty(key).trim());
                    if (!Double.isNaN(x) && (x < rows[i].min || x > rows[i].max)) throw new IllegalArgumentException("out of range");
                    v[i] = x;
                } catch (RuntimeException ex) { skipped.append(skipped.length() == 0 ? "" : ", ").append(key); }
            }
            System.arraycopy(v, 0, saved, 0, v.length);
            return skipped.toString();
        }
        public void markSaved() { System.arraycopy(v, 0, saved, 0, v.length); }
        public String text(int i) { return format(rows[i].key, v[i]); }
        /** e.g. "Launcher ticks/s: 1200   ** CHANGED (default 1180) **" */
        public String line(int i) {
            String s = rows[i].label + ": " + text(i);
            if (changed(i)) s += "   ** CHANGED (default " + format(rows[i].key, def(i)) + ")"
                    + (NOT_SAVED.equals(rows[i].key) ? ", this run only" : unsaved(i) ? ", NOT SAVED" : "") + " **";
            else if (unsaved(i)) s += "   (back to default, Square to save)";
            return s;
        }
        /** "Side=RED, Launcher ticks/s=1200" or "" */
        public String changedSummary() {
            StringBuilder b = new StringBuilder();
            for (int i = 0; i < v.length; i++)
                if (changed(i)) b.append(b.length() == 0 ? "" : ", ").append(rows[i].label).append('=').append(text(i));
            return b.toString();
        }
    }

    public static boolean same(double a, double b) { return Double.isNaN(a) ? Double.isNaN(b) : Math.abs(a - b) < 1e-9; }

    public static String format(String key, double x) {
        if (key.equals("execution")) return x == 0 ? "DRY (no motors)" : x == 1 ? "DRIVE (drivetrain only)" : "LIVE";
        if (key.equals("side")) return x == 1 ? "RED" : x == 2 ? "BLUE" : "AUTO (detect)";
        if (key.equals("park")) return x == 0 ? "no" : "yes";
        if (Double.isNaN(x)) return "NOT SET";
        if (x == Math.rint(x) && Math.abs(x) < 1e6) return String.format(Locale.US, "%.0f", x);
        return String.format(Locale.US, "%.2f", x);
    }
}
