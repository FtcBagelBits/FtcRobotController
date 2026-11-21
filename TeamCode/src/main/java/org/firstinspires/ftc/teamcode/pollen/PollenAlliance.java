package org.firstinspires.ftc.teamcode.pollen;

import java.util.Locale;
import java.util.Properties;

/** The alliance last chosen in an autonomous INIT, handed to TeleOp so Circle aims only at our
 * alliance's hive cells. Every hive auto writes FIRST/pollen-hive/alliance.properties when RED or
 * BLUE is selected and again at START; the newest write wins. Pure Java (PollenHiveStore does the
 * file work) so the desktop tests can check it. */
public final class PollenAlliance {
    private PollenAlliance() { }
    public static final int RED = 1, BLUE = 2;
    /** Both hive cells of one alliance (first tag IDs, see IsHiveReadyForShot), or every cell. */
    public static int[] cells(int side) {
        return side == RED ? new int[]{30, 34} : side == BLUE ? new int[]{38, 42} : IsHiveReadyForShot.ALL_CELLS.clone();
    }
    public static String name(int side) { return side == RED ? "RED" : side == BLUE ? "BLUE" : "ANY"; }

    /** One saved choice: which alliance, when (Hub clock, ms) and which OpMode chose it. */
    public static final class Saved {
        public final int side;
        public final long wallMs;
        public final String source;
        Saved(int side, long wallMs, String source) { this.side = side; this.wallMs = wallMs; this.source = source; }
        /** For the TeleOp INIT screen, e.g. "BLUE, from PollenHivePrimaryAngledAuto 12 min ago". */
        public String describe(long nowMs) {
            long minutes = Math.max(0, (nowMs - wallMs) / 60000);
            String age = minutes < 1 ? "just now" : minutes < 120 ? minutes + " min ago"
                    : String.format(Locale.US, "%.1f h ago", minutes / 60.0);
            return name(side) + ", from " + source + " " + age;
        }
    }

    static Properties toProperties(int side, long wallMs, String source) {
        if (side != RED && side != BLUE) throw new IllegalArgumentException("Alliance must be RED or BLUE");
        Properties p = new Properties();
        p.setProperty("alliance", name(side));
        p.setProperty("wall_ms", Long.toString(wallMs));
        p.setProperty("source", source == null ? "unknown" : source);
        p.setProperty("build", PollenConfig.BUILD);
        return p;
    }

    /** The saved choice, or null if the file is missing, damaged or names no alliance. */
    static Saved parse(Properties p) {
        if (p == null) return null;
        String a = p.getProperty("alliance", "");
        int side = a.equals("RED") ? RED : a.equals("BLUE") ? BLUE : 0;
        if (side == 0) return null;
        long wall;
        try { wall = Long.parseLong(p.getProperty("wall_ms", "0")); } catch (NumberFormatException ex) { wall = 0; }
        return new Saved(side, wall, p.getProperty("source", "unknown"));
    }
}
