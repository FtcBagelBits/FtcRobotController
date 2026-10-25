package org.firstinspires.ftc.teamcode.pollen;

/** Nominal routes derived from official BIOBUZZ STEP v26-27.2 for an 18x18-inch
 * robot with a centered launcher. Both shooters start flush and square to an end
 * wall; nothing turns the chassis. These are the controller presets' defaults only.
 * See docs/POLLEN_FIELD_ROUTES.md for coordinates, margins and assumptions. */
public final class PollenFieldGeometry {
    private PollenFieldGeometry() { }
    public static final double INCH_TO_CM = 2.54;
    public static final double ROBOT_HALF_INCHES = 9;
    public static final double END_WALL_INCHES = 70.585;
    public static final double RED_HIVE_X_INCHES = -12.74164098;
    public static final double PARK_X_INCHES = -56;
    public static final double PRIMARY_PARK_Z_INCHES = -26;
    public static final double SUPPORT_PARK_Z_INCHES = -47;
    // Inboard edge of the scoring-wall flower's all-height CAD envelope (its wall bracket).
    public static final double SCORING_FLOWER_INBOARD_X_INCHES = -20.058;
    public static final double ALLIANCE_MIDLINE_X_INCHES = 0;
    // Support: flush and square, centered in the flower-to-midline gap (equal margins).
    public static final double SUPPORT_START_X_INCHES =
            (SCORING_FLOWER_INBOARD_X_INCHES + ALLIANCE_MIDLINE_X_INCHES) / 2;
    public static final double PRIMARY_WALL_CLEARANCE_CM = 10;
    public static final double PRIMARY_SIDE_CLEARANCE_INCHES = 25;
    public static final double SUPPORT_WALL_CLEARANCE_INCHES = 8;
    private static final double START_Z = END_WALL_INCHES - ROBOT_HALF_INCHES;
    private static final double LEFT_TO_PARK = RED_HIVE_X_INCHES - PARK_X_INCHES;

    /** Primary leg 3: after forward wall clearance and the left hive clearance. */
    public static double primaryDistanceCm() {
        return Math.hypot(primaryLeftRemainingInches(), primaryForwardRemainingInches()) * INCH_TO_CM;
    }
    public static double primaryAngleDegrees() {
        return Math.toDegrees(Math.atan2(primaryLeftRemainingInches(), primaryForwardRemainingInches()));
    }
    private static double primaryLeftRemainingInches() {
        return LEFT_TO_PARK - PRIMARY_SIDE_CLEARANCE_INCHES;
    }
    private static double primaryForwardRemainingInches() {
        return START_Z - PRIMARY_WALL_CLEARANCE_CM / INCH_TO_CM - PRIMARY_PARK_Z_INCHES;
    }

    /** Support leg 3: after forward flower clearance; robot-right is field -X. */
    public static double supportDistanceCm() {
        return Math.hypot(supportRightRemainingInches(), supportForwardRemainingInches()) * INCH_TO_CM;
    }
    public static double supportAngleDegrees() {
        return -Math.toDegrees(Math.atan2(supportRightRemainingInches(), supportForwardRemainingInches()));
    }
    private static double supportRightRemainingInches() {
        return SUPPORT_START_X_INCHES - PARK_X_INCHES;
    }
    private static double supportForwardRemainingInches() {
        return SUPPORT_PARK_Z_INCHES - supportClearanceZInches();
    }
    public static double supportStartZInches() { return -START_Z; }
    public static double supportClearanceZInches() {
        return supportStartZInches() + SUPPORT_WALL_CLEARANCE_INCHES;
    }

    /** Lateral offset of the support robot's centerline from the red hive center. */
    public static double supportOffsetFromHiveCm() {
        return (SUPPORT_START_X_INCHES - RED_HIVE_X_INCHES) * INCH_TO_CM;
    }
    public static double supportFlowerGapCm() {
        return (SUPPORT_START_X_INCHES - ROBOT_HALF_INCHES - SCORING_FLOWER_INBOARD_X_INCHES) * INCH_TO_CM;
    }
    public static double supportMidlineGapCm() {
        return (ALLIANCE_MIDLINE_X_INCHES - SUPPORT_START_X_INCHES - ROBOT_HALF_INCHES) * INCH_TO_CM;
    }
}
