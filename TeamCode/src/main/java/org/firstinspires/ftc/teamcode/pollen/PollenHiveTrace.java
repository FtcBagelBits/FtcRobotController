package org.firstinspires.ftc.teamcode.pollen;

import java.io.*;
import java.util.Locale;

/** Bounded diagnostic CSV. Failure disables logging, never motor control. No automatic deletion.
 * One run reserves 2 MiB of a 64 MiB trace budget; keep at least 32 MiB free on the Hub. */
public final class PollenHiveTrace implements Closeable {
    static final long RUN_LIMIT = 2 * 1024 * 1024, TOTAL_LIMIT = 64 * 1024 * 1024;
    private Writer writer;
    private long bytes;
    private double flushedAt;
    public String status = "open";
    public PollenHiveTrace(File directory, String name) throws IOException {
        if (!directory.isDirectory() && !directory.mkdirs()) throw new IOException("Cannot create trace folder");
        long used = 0;
        File[] files = directory.listFiles();
        if (files == null) throw new IOException("Cannot inspect trace budget");
        for (File f : files) if (f.isFile()) used += f.length();
        if (used + RUN_LIMIT > TOTAL_LIMIT) throw new IOException("Trace budget full: download/archive traces");
        if (directory.getUsableSpace() < 32L * 1024 * 1024 + RUN_LIMIT) throw new IOException("Low disk space: trace disabled");
        File file = new File(directory, name + ".csv");
        if (!file.createNewFile()) throw new IOException("Trace already exists");
        writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(file), "UTF-8"));
    }
    // Inject a writer for failure/cap regression tests without Android or a robot.
    PollenHiveTrace(Writer writer) { this.writer = writer; }
    public void row(double now, boolean flush, Object... values) {
        if (writer == null) return;
        StringBuilder line = new StringBuilder();
        for (Object value : values) {
            if (line.length() > 0) line.append(',');
            String s = value instanceof Double ? String.format(Locale.US, "%.4f", value) : String.valueOf(value);
            line.append('"').append(s.replace("\"", "\"\"")).append('"');
        }
        line.append('\n');
        try {
            int size = line.toString().getBytes("UTF-8").length;
            if (bytes + size > RUN_LIMIT) { status = "Trace run limit reached"; close(); return; }
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
