package org.firstinspires.ftc.teamcode.pollen;

import java.io.File;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.io.StringWriter;
import java.io.Writer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/** The TeleOp recorder, rehearsal marks, and the storage budgets. No robot and no SDK: files go
 * to a temporary folder on this computer. */
public final class PollenRecordingTest {
    private static int checks;

    private static void check(boolean ok, String why) {
        checks++;
        if (!ok) throw new AssertionError(why);
    }

    /** The rows of a CSV written by PollenHiveTrace (every value is quoted; none here contain commas). */
    private static List<String[]> rows(StringWriter out) {
        List<String[]> rows = new ArrayList<>();
        for (String line : out.toString().split("\n")) {
            if (line.isEmpty()) continue;
            String[] cells = line.substring(1, line.length() - 1).split("\",\"", -1);
            rows.add(cells);
        }
        return rows;
    }

    private static int column(List<String[]> rows, String name) {
        String[] header = rows.get(0);
        for (int i = 0; i < header.length; i++) if (header[i].equals(name)) return i;
        throw new AssertionError("No column " + name);
    }

    private static int count(List<String[]> rows, String kind) {
        int kindColumn = column(rows, "kind"), n = 0;
        for (String[] row : rows) if (row[kindColumn].equals(kind)) n++;
        return n;
    }

    private static void sampleRate() {
        StringWriter out = new StringWriter();
        PollenTeleOpRecorder recorder = new PollenTeleOpRecorder(new PollenHiveTrace(out));
        PollenTeleOpRecorder.Sample s = new PollenTeleOpRecorder.Sample();
        recorder.start(1000, "build=test launcher=1180", s);
        // 10 idle seconds at 100 loops a second: about one row a second, not 1000.
        for (int i = 1; i <= 1000; i++) recorder.loop(i * 0.01, 1000 + i * 10, false, false, s);
        List<String[]> rows = rows(out);
        int idle = count(rows, "SAMPLE");
        check(idle >= 9 && idle <= 11, "Idle robot: about one row a second, got " + idle);
        check(rows.get(1)[column(rows, "kind")].equals("START")
                && rows.get(1)[column(rows, "note")].equals("build=test launcher=1180"), "START row carries the settings");
        check(rows.get(0).length == rows.get(1).length, "Every row has a value for every column");
        // Now the driver pushes the stick: 20 rows a second.
        s.stickForward = 0.5;
        s.wheels[0] = s.wheels[1] = s.wheels[2] = s.wheels[3] = 0.5;
        for (int i = 1001; i <= 1200; i++) recorder.loop(i * 0.01, 1000 + i * 10, false, false, s);
        rows = rows(out);
        int driving = count(rows, "SAMPLE") - idle;
        check(driving >= 38 && driving <= 41, "Driving: about 20 rows a second, got " + driving);
        String[] last = rows.get(rows.size() - 1);
        check(last[column(rows, "wheel_fl")].equals("0.5000") && last[column(rows, "stick_forward")].equals("0.5000"),
                "Rows hold the driver input and the wheel power");
        check(last[column(rows, "x_in")].equals("NaN") && last[column(rows, "odom_status")].equals("NO PINPOINT"),
                "Without odometry the position is NaN, not zero");
        // A coasting launcher still counts as activity, so the spin-down is recorded.
        s.stickForward = 0;
        java.util.Arrays.fill(s.wheels, 0);
        s.launcherMeasured = 600;
        int before = count(rows(out), "SAMPLE");
        for (int i = 1201; i <= 1300; i++) recorder.loop(i * 0.01, 0, false, false, s);
        check(count(rows(out), "SAMPLE") - before >= 19, "Launcher spin-down is recorded at the fast rate");
        recorder.stop(13.01, 0, s);
        recorder.stop(13.02, 0, s);
        check(count(rows(out), "STOP") == 1, "STOP is written once");
    }

    private static void rehearsals() {
        StringWriter out = new StringWriter();
        PollenTeleOpRecorder recorder = new PollenTeleOpRecorder(new PollenHiveTrace(out));
        PollenTeleOpRecorder.Sample s = new PollenTeleOpRecorder.Sample();
        s.odometry = "READY";
        recorder.start(0, "settings", s);
        check(recorder.screen(0).contains("L1 = start a rehearsal"), "Screen says how to start");
        check(recorder.loop(0.02, 0, false, true, s) == PollenTeleOpRecorder.Mark.NONE,
                "Square with no rehearsal open does nothing");
        recorder.loop(0.04, 0, false, false, s);
        s.x = 12; s.y = -3; s.headingDegrees = 90;
        check(recorder.loop(1.00, 0, true, false, s) == PollenTeleOpRecorder.Mark.STARTED, "L1 starts rehearsal 1");
        check(recorder.loop(1.02, 0, true, false, s) == PollenTeleOpRecorder.Mark.NONE, "Holding L1 does not start another");
        check(recorder.rehearsal() == 1 && recorder.screen(3).contains("REHEARSAL 1: 2.0 s"), "Screen shows the open rehearsal");
        // A still robot inside a rehearsal is still recorded at the fast rate (a pause is part of the move).
        int before = count(rows(out), "SAMPLE");
        for (int i = 0; i < 100; i++) recorder.loop(1.04 + i * 0.01, 0, false, false, s);
        check(count(rows(out), "SAMPLE") - before >= 19, "Rehearsal rows are written even while the robot is still");
        check(recorder.loop(2.10, 0, false, true, s) == PollenTeleOpRecorder.Mark.ENDED, "Square ends it");
        check(recorder.rehearsal() == 0, "No rehearsal open after Square");
        recorder.loop(2.12, 0, false, false, s);
        check(recorder.loop(3.00, 0, true, false, s) == PollenTeleOpRecorder.Mark.STARTED, "L1 starts rehearsal 2");
        recorder.loop(3.02, 0, false, false, s);
        check(recorder.loop(4.00, 0, true, false, s) == PollenTeleOpRecorder.Mark.STARTED, "L1 again: rehearsal 3");
        recorder.event(4.5, 0, "AIM_DONE n=1 source=CAMERA", s);
        recorder.stop(5, 0, s);
        List<String[]> rows = rows(out);
        check(count(rows, "MARK_START") == 3 && count(rows, "MARK_END") == 3, "Every start has an end");
        check(count(rows, "AIM") == 1, "Circle's aim writes an AIM row");
        int kind = column(rows, "kind"), number = column(rows, "rehearsal"), note = column(rows, "note");
        StringBuilder marks = new StringBuilder();
        for (String[] row : rows) if (row[kind].startsWith("MARK")) marks.append(row[kind]).append(row[number]).append(' ');
        check(marks.toString().equals("MARK_START1 MARK_END1 MARK_START2 MARK_END2 MARK_START3 MARK_END3 "),
                "Marks are numbered in order: " + marks);
        for (String[] row : rows) {
            if (row[kind].equals("MARK_START") && row[number].equals("1"))
                check(row[column(rows, "x_in")].equals("12.0000") && row[column(rows, "heading_deg")].equals("90.0000"),
                        "MARK_START holds the pose where L1 was pressed");
            if (row[kind].equals("MARK_END") && row[number].equals("2")) check(row[note].contains("next L1"), "End reason: next L1");
            if (row[kind].equals("MARK_END") && row[number].equals("3")) check(row[note].contains("STOP"), "End reason: STOP");
        }
        check(rows.get(rows.size() - 1)[kind].equals("STOP"), "STOP is the last row");
    }

    private static void neverStopsTheRobot() {
        PollenTeleOpRecorder.Sample s = new PollenTeleOpRecorder.Sample();
        // No file at all (budget full, no storage): everything is a quiet no-op.
        PollenTeleOpRecorder none = new PollenTeleOpRecorder(null);
        none.start(0, "x", s);
        check(none.loop(1, 0, true, false, s) == PollenTeleOpRecorder.Mark.STARTED, "Marks still work without a file");
        none.stop(2, 0, s);
        check(none.screen(1).contains("NOT RECORDING"), "Screen says it is not recording");
        // A disk that fails: the trace reports it and later rows are skipped.
        Writer broken = new Writer() {
            public void write(char[] c, int o, int n) throws IOException { throw new IOException("disk unavailable"); }
            public void flush() { }
            public void close() { }
        };
        PollenHiveTrace trace = new PollenHiveTrace(broken);
        PollenTeleOpRecorder recorder = new PollenTeleOpRecorder(trace);
        recorder.start(0, "x", s);
        for (int i = 1; i < 50; i++) recorder.loop(i * 0.1, 0, i == 5, false, s);
        recorder.stop(5, 0, s);
        check(recorder.screen(5).contains("STOPPED: Trace failed"), "Screen shows the write failure: " + recorder.screen(5));
        // The per-run limit closes the file instead of filling the Hub.
        StringWriter out = new StringWriter();
        trace = new PollenHiveTrace(out, 4000);
        recorder = new PollenTeleOpRecorder(trace);
        recorder.start(0, "x", s);
        s.stickForward = 1;
        s.wheels[0] = 1;
        for (int i = 1; i < 2000; i++) recorder.loop(i * 0.05, 0, false, false, s);
        check(out.toString().length() <= 4000 && trace.status.contains("limit"), "One run cannot grow past its limit");
        check(recorder.screen(0).contains("STOPPED"), "Screen shows the limit was reached");
    }

    private static void storage() throws Exception {
        Path root = Files.createTempDirectory("pollen-run-log-test");
        try {
            PollenRunLog.Storage empty = PollenRunLog.storage(root.resolve("not-created-yet").toFile());
            check(empty.freeBytes > 0, "Free space is read from the parent when the folder does not exist yet");
            check(empty.warning().isEmpty() || empty.warning().startsWith("HUB STORAGE LOW"), "No folder warnings when empty");
            check(empty.line().contains("teleop 0/128 MB") && empty.line().contains("traces 0/64 MB")
                    && empty.line().contains("GB free"), "Storage line lists each folder and its budget: " + empty.line());

            PollenHiveTrace first = PollenRunLog.open(root.toFile(), PollenRunLog.Kind.TELEOP, 1763300000000L);
            PollenHiveTrace second = PollenRunLog.open(root.toFile(), PollenRunLog.Kind.TELEOP, 1763300000000L);
            first.row(0, true, "a");
            first.close();
            second.close();
            File[] files = root.resolve("teleop").toFile().listFiles();
            check(files != null && files.length == 2, "Two runs in the same second get two files");
            boolean named = false, numbered = false;
            for (File f : files) {
                named |= f.getName().matches("teleop-\\d{8}-\\d{6}\\.csv");
                numbered |= f.getName().matches("teleop-\\d{8}-\\d{6}-2\\.csv");
            }
            check(named && numbered, "Files are named for the clock, with -2 for a second one");
            check(PollenRunLog.storage(root.toFile()).line().contains("teleop 1/128 MB"), "A used folder rounds up to 1 MB");

            // Nearly full: warn. Full: refuse a new recording, and keep the old file.
            File big = root.resolve("auto").resolve("old.csv").toFile();
            check(big.getParentFile().mkdirs(), "Test folder created");
            try (RandomAccessFile f = new RandomAccessFile(big, "rw")) { f.setLength(27 * PollenRunLog.MIB); }
            check(PollenRunLog.storage(root.toFile()).warning().contains("auto logs nearly full"), "80% of the budget warns");
            PollenRunLog.open(root.toFile(), PollenRunLog.Kind.AUTO, 0).close();
            try (RandomAccessFile f = new RandomAccessFile(big, "rw")) { f.setLength(PollenRunLog.Kind.AUTO.totalLimit); }
            PollenRunLog.Storage full = PollenRunLog.storage(root.toFile());
            check(full.warning().contains("auto log budget FULL - NOT RECORDING") && full.warning().contains("download, then prune"),
                    "A full budget says so and says what to do: " + full.warning());
            boolean refused = false;
            try { PollenRunLog.open(root.toFile(), PollenRunLog.Kind.AUTO, 1000); }
            catch (IOException ex) { refused = ex.getMessage().contains("budget"); }
            check(refused && big.length() == PollenRunLog.Kind.AUTO.totalLimit, "Full budget refuses a new file and deletes nothing");
            check(new PollenRunLog.Storage(100 * PollenRunLog.MIB, new long[PollenRunLog.Kind.values().length]).warning()
                    .startsWith("HUB STORAGE LOW (100 MB free)"), "Low free space warns");
        } finally {
            deleteTree(root.toFile());
        }
    }

    private static void deleteTree(File file) {
        File[] children = file.listFiles();
        if (children != null) for (File child : children) deleteTree(child);
        if (!file.delete()) throw new AssertionError("Could not delete " + file);
    }

    public static void main(String[] args) throws Exception {
        sampleRate();
        rehearsals();
        neverStopsTheRobot();
        storage();
        System.out.println("PASS PollenRecordingTest: " + checks + " checks");
    }
}
