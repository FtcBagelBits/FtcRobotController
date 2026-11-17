package org.firstinspires.ftc.teamcode.pollen;

import java.io.*;
import java.util.Locale;

/** Bounded diagnostic CSV. Failure disables logging, never motor control. No automatic deletion.
 * A hive-auto run reserves 2 MiB of a 64 MiB trace budget; keep at least 32 MiB free on the Hub.
 * The other recordings (PollenRunLog: TeleOp, match autos) use the same class with their own limits. */
public final class PollenHiveTrace implements Closeable {
    static final long RUN_LIMIT = 2 * 1024 * 1024, TOTAL_LIMIT = 64 * 1024 * 1024;
    /** No new recording starts unless this much would still be free on the Hub afterward. */
    static final long KEEP_FREE = 32L * 1024 * 1024;
    private final long runLimit;
    /** The file's name on the Hub ("" in tests that write to memory). */
    public final String fileName;
    private Writer writer;
    private long bytes;
    private double flushedAt;
    public String status = "open";
    public PollenHiveTrace(File directory, String name) throws IOException {
        this(directory, name, RUN_LIMIT, TOTAL_LIMIT);
    }
    /** runLimit: the most one file may grow to. totalLimit: the most this folder may hold. */
    public PollenHiveTrace(File directory, String name, long runLimit, long totalLimit) throws IOException {
        this.runLimit = runLimit;
        if (!directory.isDirectory() && !directory.mkdirs()) throw new IOException("Cannot create trace folder");
        long used = 0;
        File[] files = directory.listFiles();
        if (files == null) throw new IOException("Cannot inspect trace budget");
        for (File f : files) if (f.isFile()) used += f.length();
        if (used + runLimit > totalLimit) throw new IOException("Trace budget full: download/archive traces");
        if (directory.getUsableSpace() < KEEP_FREE + runLimit) throw new IOException("Low disk space: trace disabled");
        File file = new File(directory, name + ".csv");
        fileName = file.getName();
        if (!file.createNewFile()) throw new IOException("Trace already exists");
        writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(file), "UTF-8"));
    }
    // Inject a writer for failure/cap regression tests without Android or a robot.
    PollenHiveTrace(Writer writer) { this(writer, RUN_LIMIT); }
    PollenHiveTrace(Writer writer, long runLimit) { this.writer = writer; this.runLimit = runLimit; fileName = ""; }
    /** Bytes written to this file so far. */
    public long bytes() { return bytes; }
    public void row(double now, boolean flush, Object... values) {
        if (writer == null) return;
        StringBuilder line = new StringBuilder();
        for (Object value : values) {
            if (line.length() > 0) line.append(',');
            // Adding 0.0 turns -0.0 into 0.0, so a centered stick never prints as "-0.0000".
            String s = value instanceof Double ? String.format(Locale.US, "%.4f", (Double) value + 0.0) : String.valueOf(value);
            line.append('"').append(s.replace("\"", "\"\"")).append('"');
        }
        line.append('\n');
        try {
            int size = line.toString().getBytes("UTF-8").length;
            if (bytes + size > runLimit) { status = "Trace run limit reached"; close(); return; }
            writer.write(line.toString()); bytes += size;
            if (flush || now - flushedAt >= 1) { writer.flush(); flushedAt = now; }
        } catch (IOException ex) { status = "Trace failed: " + ex; close(); }
    }
    @Override public void close() {
        if (writer != null) try { writer.close(); }
        catch (IOException ex) { status = "Trace close failed: " + ex; }
        finally { writer = null; }
    }
}
