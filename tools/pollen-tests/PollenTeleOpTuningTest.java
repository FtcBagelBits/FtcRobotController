import org.firstinspires.ftc.teamcode.pollen.PollenConfig;
import org.firstinspires.ftc.teamcode.pollen.PollenTeleOpTuning;
import static org.firstinspires.ftc.teamcode.pollen.PollenTeleOpTuning.Row.*;

/** PollenBot TeleOp INIT menu: defaults, edits, limits, memory, locking and the two feeding rules. */
public final class PollenTeleOpTuningTest {
    private static int checks;
    private static void check(boolean ok, String message) {
        checks++;
        if (!ok) throw new AssertionError(message);
    }
    private static void near(double actual, double expected, String message) {
        check(Math.abs(actual - expected) < 1e-9, message + ": " + actual + " != " + expected);
    }
    private static void release(PollenTeleOpTuning t) { t.update(false, false, false, false, false); }
    private static void tap(PollenTeleOpTuning t, int direction, int count) {
        for (int i = 0; i < count; i++) { t.update(false, false, direction < 0, direction > 0, false); release(t); }
    }
    private static void select(PollenTeleOpTuning t, PollenTeleOpTuning.Row row) {
        while (t.selected() != row) { t.update(false, true, false, false, false); release(t); }
    }

    public static void main(String[] args) {
        PollenTeleOpTuning t = new PollenTeleOpTuning(1250, PollenConfig.LAUNCHER_SPINUP_SECONDS,
                PollenConfig.AGITATOR_FEED_POWER, null);
        near(t.launcherVelocity(), 1250, "default launcher target is the TeleOp constant");
        near(t.spinupSeconds(), 3, "default spin-up is 3 s");
        near(t.agitatorPower(), 1, "agitator runs at full power while feeding by default");
        near(t.driveScale(), 1, "default drive power leaves sticks unchanged");
        check(t.isDefault() && t.selected() == LAUNCHER, "starts on launcher row with defaults");

        // Feeding starts at rule 1 (up to speed) OR rule 2 (spin-up time over); TeleOp ORs them.
        near(t.launcherReadyVelocity(), 1200, "up to speed is the original 1200 ticks/s");
        check(t.launcherIsUpToSpeed(1200) && t.launcherIsUpToSpeed(1300), "1200 and above is up to speed");
        check(!t.launcherIsUpToSpeed(1199) && !t.launcherIsUpToSpeed(0), "below 1200 is not");
        check(!t.launcherIsUpToSpeed(Double.NaN), "a bad reading never counts as up to speed");
        check(!t.spinUpTimeIsOver(0) && !t.spinUpTimeIsOver(2.99), "backup timer not over before 3 s");
        check(t.spinUpTimeIsOver(3) && t.spinUpTimeIsOver(10), "backup timer over from 3 s on");

        tap(t, 1, 3); near(t.launcherVelocity(), 1400, "launcher +50 per tap");
        near(t.launcherReadyVelocity(), 1344, "up-to-speed level follows the edited target");
        t.update(false, false, false, true, false); t.update(false, false, false, true, false);
        near(t.launcherVelocity(), 1450, "held button does not repeat"); release(t);
        t.update(false, false, true, true, false); near(t.launcherVelocity(), 1450, "opposite arrows cancel"); release(t);
        tap(t, 1, 100); near(t.launcherVelocity(), 2500, "launcher upper limit");
        tap(t, -1, 100); near(t.launcherVelocity(), 500, "launcher lower limit");

        select(t, SPINUP); near(t.launcherVelocity(), 500, "changing rows does not edit");
        tap(t, -1, 2); near(t.spinupSeconds(), 2, "spin-up -0.5 s per tap");
        check(t.spinUpTimeIsOver(2) && !t.spinUpTimeIsOver(1.99), "backup timer follows the edited spin-up");
        tap(t, -1, 20); near(t.spinupSeconds(), 0, "spin-up lower limit (feed at once)");
        check(t.spinUpTimeIsOver(0), "zero spin-up feeds with R1");
        tap(t, 1, 20); near(t.spinupSeconds(), 6, "spin-up upper limit");

        select(t, AGITATOR);
        tap(t, 1, 1); near(t.agitatorPower(), 1, "agitator cannot exceed full power");
        tap(t, -1, 2); near(t.agitatorPower(), .9, "agitator -0.05 per tap");
        tap(t, -1, 100); near(t.agitatorPower(), .1, "agitator lower limit");

        select(t, DRIVE);
        tap(t, 1, 1); near(t.driveScale(), 1, "drive cannot exceed 100%");
        tap(t, -1, 4); near(t.driveScale(), .8, "drive -5% per tap");
        tap(t, -1, 100); near(t.driveScale(), .2, "drive lower limit");
        t.update(false, true, false, false, false); release(t);
        check(t.selected() == LAUNCHER, "DOWN wraps from the last row to the first");
        check(!t.isDefault(), "changed settings reported");

        PollenTeleOpTuning kept = new PollenTeleOpTuning(1250, 3, 1, t.snapshot());
        near(kept.launcherVelocity(), 500, "launcher kept for next INIT");
        near(kept.spinupSeconds(), 6, "spin-up kept for next INIT");
        near(kept.agitatorPower(), .1, "agitator power kept for next INIT");
        near(kept.driveScale(), .2, "drive kept for next INIT");
        tap(t, 1, 1); near(kept.launcherVelocity(), 500, "snapshots do not alias");

        t.update(false, false, false, false, true); release(t);
        check(t.isDefault(), "Triangle restores all defaults");
        near(t.launcherVelocity(), 1250, "reset launcher"); near(t.spinupSeconds(), 3, "reset spin-up");
        near(t.driveScale(), 1, "reset drive"); near(t.agitatorPower(), 1, "reset agitator");
        t.update(false, false, false, false, true); t.update(false, false, true, false, true);
        near(t.launcherVelocity(), 1250, "reset dominates and does not repeat");
        release(t);

        tap(t, 1, 2); t.lock(); tap(t, 1, 5); t.update(false, false, false, false, true);
        near(t.launcherVelocity(), 1350, "START locks edits and reset");
        check(t.locked(), "locked flag");

        boolean rejected = false;
        try { new PollenTeleOpTuning(1250, -1, 1, null); } catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "negative spin-up rejected");
        System.out.println("PollenTeleOpTuningTest: " + checks + " checks passed");
    }
}
