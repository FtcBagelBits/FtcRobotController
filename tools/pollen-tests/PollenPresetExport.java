import org.firstinspires.ftc.teamcode.pollen.PollenFieldGeometry;
import org.firstinspires.ftc.teamcode.pollen.PollenShootTuning;

/** Prints the field presets exactly as the Java code builds them, as JSON, so
 * tools/check-pollen-field-routes.py checks the real routes rather than its own copy.
 * Leg angles are desired physical directions (MAT_TRIAL), + = robot left.
 * "plan_legs" is the CAD plan from PollenFieldGeometry; "legs" is what the preset loads.
 * Leg 3 of both presets is field-tuned, so it is reported, not required to match the plan. */
public final class PollenPresetExport {
    private static String legs(PollenShootTuning t) {
        StringBuilder b = new StringBuilder("[");
        for (int leg = 1; leg <= PollenShootTuning.LEGS; leg++) {
            if (leg > 1) b.append(", ");
            b.append(String.format(java.util.Locale.US, "[%.10f, %.10f]", t.legCm(leg), t.legAngle(leg)));
        }
        return b.append("]").toString();
    }

    private static String plan(double[][] legs) {
        StringBuilder b = new StringBuilder("[");
        for (int i = 0; i < legs.length; i++)
            b.append(i > 0 ? ", " : "").append(String.format(java.util.Locale.US, "[%.10f, %.10f]", legs[i][0], legs[i][1]));
        return b.append("]").toString();
    }

    public static void main(String[] args) {
        PollenShootTuning b = new PollenShootTuning(null);
        b.update(false, false, false, false, false, true);  // Square: B field preset
        PollenShootTuning c = new PollenShootTuning(null, true);
        double startZ = PollenFieldGeometry.END_WALL_INCHES - PollenFieldGeometry.ROBOT_HALF_INCHES;
        String bPlan = plan(new double[][] {{PollenFieldGeometry.PRIMARY_WALL_CLEARANCE_CM, 0},
                {PollenFieldGeometry.PRIMARY_SIDE_CLEARANCE_INCHES * PollenFieldGeometry.INCH_TO_CM, 90},
                {PollenFieldGeometry.primaryDistanceCm(), PollenFieldGeometry.primaryAngleDegrees()}});
        String cPlan = plan(new double[][] {{PollenFieldGeometry.SUPPORT_WALL_CLEARANCE_INCHES * PollenFieldGeometry.INCH_TO_CM, 0},
                {0, -90}, {PollenFieldGeometry.supportDistanceCm(), PollenFieldGeometry.supportAngleDegrees()}});
        System.out.println(String.format(java.util.Locale.US,
                "{\"wall\": %.10f, \"half\": %.10f, \"red_hive_x\": %.10f, \"flower_inboard_x\": %.10f,%n"
                        + " \"b_square\": {\"start\": [%.10f, %.10f], \"facing\": \"-Z\", \"park\": [%.10f, %.10f], \"legs\": %s, \"plan_legs\": %s, \"field_tuned_legs\": [3]},%n"
                        + " \"c\": {\"start\": [%.10f, %.10f], \"facing\": \"+Z\", \"park\": [%.10f, %.10f], \"legs\": %s, \"plan_legs\": %s, \"field_tuned_legs\": [3]}}",
                PollenFieldGeometry.END_WALL_INCHES, PollenFieldGeometry.ROBOT_HALF_INCHES,
                PollenFieldGeometry.RED_HIVE_X_INCHES, PollenFieldGeometry.SCORING_FLOWER_INBOARD_X_INCHES,
                PollenFieldGeometry.RED_HIVE_X_INCHES, startZ,
                PollenFieldGeometry.PARK_X_INCHES, PollenFieldGeometry.PRIMARY_PARK_Z_INCHES, legs(b), bPlan,
                PollenFieldGeometry.SUPPORT_START_X_INCHES, -startZ,
                PollenFieldGeometry.PARK_X_INCHES, PollenFieldGeometry.SUPPORT_PARK_Z_INCHES, legs(c), cPlan));
    }
}
