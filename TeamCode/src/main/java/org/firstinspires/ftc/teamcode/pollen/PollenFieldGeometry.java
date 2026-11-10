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
    /** Estimated middle of the hive cell's opening, this far from the field center toward the
     * shooter (docs/POLLEN_FIELD_ROUTES.md). From the wall start it is straight ahead, 45.9 in. */
    public static final double HIVE_OPENING_Z_INCHES = 15.7;
    /** EXPERIMENTAL angled shot: the team's ASSUMED shooting spot (strategy canvas, 2026-11-08),
     * as the robot's center measured from where it starts against the wall: this far forward
     * and this far to its left. Not yet confirmed on a field. */
    public static final double ANGLED_OFF_WALL_INCHES = 5, ANGLED_SIDE_INCHES = 30;
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

    /** Angled shot: heading that points the robot at the middle of the hive opening from the
     * shooting spot, in degrees (+left). The spot is to the LEFT of the hive, so the answer is
     * negative (turn right): about -36 degrees, with the opening about 51 in away. */
    public static double angledAimDegrees() {
        return -Math.toDegrees(Math.atan2(ANGLED_SIDE_INCHES, angledForwardToOpeningInches()));
    }
    public static double angledShotDistanceInches() {
        return Math.hypot(ANGLED_SIDE_INCHES, angledForwardToOpeningInches());
    }
    private static double angledForwardToOpeningInches() {
        return START_Z - HIVE_OPENING_Z_INCHES - ANGLED_OFF_WALL_INCHES;
    }
    /** Angled leg 3: from the shooting spot to the same parking spot as the primary route. */
    public static double angledDistanceCm() {
        return Math.hypot(angledLeftRemainingInches(), angledForwardRemainingInches()) * INCH_TO_CM;
    }
    public static double angledAngleDegrees() {
        return Math.toDegrees(Math.atan2(angledLeftRemainingInches(), angledForwardRemainingInches()));
    }
    private static double angledLeftRemainingInches() {
        return LEFT_TO_PARK - ANGLED_SIDE_INCHES;
    }
    private static double angledForwardRemainingInches() {
        return START_Z - ANGLED_OFF_WALL_INCHES - PRIMARY_PARK_Z_INCHES;
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
