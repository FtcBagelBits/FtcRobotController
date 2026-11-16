package org.firstinspires.ftc.teamcode.pollen;

import java.io.*;
import java.util.Properties;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/** Saved CHANGES (never full copies) on the Control Hub, in FIRST/pollen-hive (normally
 * /sdcard/FIRST/pollen-hive): camera.properties (all hive modes), primary.properties,
 * secondary.properties. Pull the folder to promote changes to code defaults:
 *   adb pull /sdcard/FIRST/pollen-hive ~/Desktop/pollen-hive
 *   python3 tools/hive-promote-defaults.py ~/Desktop/pollen-hive */
public final class PollenHiveStore {
    private PollenHiveStore() { }
    public static final String CAMERA = "camera", PRIMARY = "primary", SECONDARY = "secondary";
    public static File folder() { return new File(AppUtil.FIRST_FOLDER, "pollen-hive"); }
    private static File file(String name) { return new File(folder(), name + ".properties"); }

    /** One compact text report per run, alongside the saved settings. */
    public static File newReport(String kind) throws IOException {
        File dir = new File(folder(), "reports");
        if (!dir.isDirectory() && !dir.mkdirs()) throw new IOException("Cannot create " + dir);
        File report = File.createTempFile(kind + "-" + System.currentTimeMillis() + "-", ".txt", dir);
        appendReport(report, 0, "build=" + PollenConfig.BUILD + " run=" + kind);
        return report;
    }
    public static void appendReport(File report, double seconds, String text) throws IOException {
        try (Writer out = new OutputStreamWriter(new FileOutputStream(report, true), "UTF-8")) {
            out.write(String.format(java.util.Locale.US, "%.3fs %s%n", seconds, text));
        }
    }

    /** Defaults plus saved changes. Returns a message for telemetry. */
    public static String load(String name, PollenHiveSettings.Values values) {
        File f = file(name);
        if (!f.isFile()) return "no saved changes";
        Properties p = new Properties();
        try (InputStream in = new FileInputStream(f)) { p.load(in); }
        catch (IOException ex) { return "COULD NOT READ " + f + ": " + ex.getMessage(); }
        String skipped = values.load(p);
        return skipped.isEmpty() ? "saved changes loaded" : "saved changes loaded; IGNORED unknown/invalid: " + skipped;
    }

    /** Write only the rows that differ from their defaults (an empty file = all defaults). */
    public static void save(String name, PollenHiveSettings.Values values) throws IOException {
        File dir = folder();
        if (!dir.isDirectory() && !dir.mkdirs()) throw new IOException("Cannot create " + dir);
        File dest = file(name), temp = new File(dest.getPath() + ".tmp");
        try (FileOutputStream out = new FileOutputStream(temp)) {
            values.overrides().store(out, "Pollen hive " + name + ": changes from the code defaults, build " + PollenConfig.BUILD);
            out.getFD().sync();
        }
        if (!temp.renameTo(dest)) throw new IOException("Cannot replace " + dest);
        values.markSaved();
    }
}
