package org.firstinspires.ftc.teamcode.pollen;

import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Where each kind of recording lives on the Control Hub, how big it may get, and how much room
 * is left. Everything is under FIRST/pollen-hive (normally /sdcard/FIRST/pollen-hive):
 *
 *   teleop/     one CSV per PollenBot TeleOp run (PollenTeleOpRecorder)
 *   auto/       one CSV per match autonomous run: the same lines PollenAutoLog sends to the robot log
 *   traces/     one CSV per hive autonomous run (PollenHiveOpMode)
 *   reports/    one small text report per hive run
 *   benchmark/  camera benchmark logs (large: up to 500 MiB for one run)
 *
 * Nothing here ever deletes a file. When a folder reaches its budget, new recordings of that kind
 * are refused and the INIT screen says so; the robot still drives. Free the space from the laptop:
 * download first, then prune (docs/POLLEN_LOGGING.md).
 *
 * Pure Java (no SDK), so it is tested on a computer.
 */
public final class PollenRunLog {
    private PollenRunLog() { }

    public static final long MIB = 1024 * 1024;
    /** Below this much free space the INIT screens warn (recording itself stops at 32 MiB). */
    public static final long LOW_FREE_BYTES = 500 * MIB;
    /** A folder this full (as a fraction of its budget) gets a "download, then prune" warning. */
    public static final double NEARLY_FULL = 0.8;

    /** One kind of recording: its folder, the most one run may write, the most the folder may hold. */
    public enum Kind {
        /** About 290 bytes a row, 20 rows a second while driving: 8 MiB is about 24 minutes of
         * nonstop driving in one run, and 128 MiB is about 6 hours in all. */
        TELEOP("teleop", 8 * MIB, 128 * MIB),
        /** About 130 bytes a line, four lines a second while moving: a 30 second auto is about 20 kB. */
        AUTO("auto", 1 * MIB, 32 * MIB),
        TRACES("traces", PollenHiveTrace.RUN_LIMIT, PollenHiveTrace.TOTAL_LIMIT),
        /** Small text files with no budget of their own. */
        REPORTS("reports", 0, 0),
        /** PollenHiveBenchmark limits one run to 500 MiB; the folder has no budget. */
        BENCHMARK("benchmark", 0, 0);

        public final String folder;
        public final long runLimit, totalLimit;
        Kind(String folder, long runLimit, long totalLimit) {
            this.folder = folder; this.runLimit = runLimit; this.totalLimit = totalLimit;
        }
    }

    /**
     * Start a new recording of this kind, named for the Hub's clock: teleop-20261116-153005.csv.
     * Throws (and records nothing) if the folder's budget is used up or the Hub is nearly full.
     */
    public static PollenHiveTrace open(File root, Kind kind, long wallMillis) throws IOException {
        File directory = new File(root, kind.folder);
        String base = kind.folder + "-" + new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date(wallMillis));
        // Two runs in the same second (or a Hub clock that went backward) must not share a file.
        String name = base;
        for (int copy = 2; new File(directory, name + ".csv").exists() && copy < 100; copy++) name = base + "-" + copy;
        return new PollenHiveTrace(directory, name, kind.runLimit, kind.totalLimit);
    }

    /** How much each folder holds and how much room the Hub has left. Reads the folders once:
     * call it in init(), not in a loop. */
    public static Storage storage(File root) {
        long[] used = new long[Kind.values().length];
        for (Kind kind : Kind.values()) used[kind.ordinal()] = used(new File(root, kind.folder));
        // A folder that does not exist yet reports 0 free; its parent (/sdcard/FIRST) does exist.
        File existing = root;
        while (existing != null && !existing.exists()) existing = existing.getParentFile();
        return new Storage(existing == null ? -1 : existing.getUsableSpace(), used);
    }

    /** Bytes in the files directly inside this folder (0 if it does not exist). */
    public static long used(File directory) {
        long bytes = 0;
        File[] files = directory.listFiles();
        if (files != null) for (File f : files) if (f.isFile()) bytes += f.length();
        return bytes;
    }

    public static final class Storage {
        /** Free bytes on the Hub's storage, or -1 if it could not be read. */
        public final long freeBytes;
        private final long[] used;

        Storage(long freeBytes, long[] used) {
            this.freeBytes = freeBytes;
            this.used = used;
        }

        public long used(Kind kind) { return used[kind.ordinal()]; }

        /** "5.21 GB free | teleop 20/128 MB, auto 1/32 MB, traces 12/64 MB, reports 0 MB, benchmark 9 MB" */
        public String line() {
            StringBuilder b = new StringBuilder(freeBytes < 0 ? "free space unknown"
                    : String.format(Locale.US, "%.2f GB free", freeBytes / 1e9));
            b.append(" |");
            for (Kind kind : Kind.values()) {
                b.append(kind.ordinal() == 0 ? " " : ", ").append(kind.folder).append(' ').append(megabytes(used(kind)));
                if (kind.totalLimit > 0) b.append('/').append(kind.totalLimit / MIB);
                b.append(" MB");
            }
            return b.toString();
        }

        /** "" when all is well; otherwise what to do, worst problem first. */
        public String warning() {
            StringBuilder b = new StringBuilder();
            if (freeBytes >= 0 && freeBytes < LOW_FREE_BYTES) {
                b.append(String.format(Locale.US, "HUB STORAGE LOW (%d MB free)", freeBytes / MIB));
            }
            for (Kind kind : Kind.values()) {
                if (kind.totalLimit == 0) continue;
                if (used(kind) + kind.runLimit > kind.totalLimit) {
                    b.append(b.length() == 0 ? "" : "; ").append(kind.folder).append(" log budget FULL - NOT RECORDING");
                } else if (used(kind) >= NEARLY_FULL * kind.totalLimit) {
                    b.append(b.length() == 0 ? "" : "; ").append(kind.folder).append(" logs nearly full");
                }
            }
            return b.length() == 0 ? "" : b + ": download, then prune (docs/POLLEN_LOGGING.md)";
        }

        private static long megabytes(long bytes) {
            return (bytes + MIB - 1) / MIB; // round up, so 1 byte never shows as 0 MB
        }
    }
}
