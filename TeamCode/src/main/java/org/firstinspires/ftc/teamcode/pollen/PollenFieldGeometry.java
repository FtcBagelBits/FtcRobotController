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
    /** Primary start (v43, match 2): flush and square on the end wall, its midline-side edge this far
     * from the center seam, which puts the launcher (left of the robot's center) on the hive's
     * center. Before v43 the robot's center was on the hive's center (edge 3.7 in from the seam). */
    public static final double PRIMARY_MIDLINE_GAP_INCHES = 1;
    public static final double PRIMARY_START_X_INCHES =
            ALLIANCE_MIDLINE_X_INCHES - PRIMARY_MIDLINE_GAP_INCHES - ROBOT_HALF_INCHES;
    /** Our hive cell's middle is this far LEFT of the primary start (looking at the hive): 2.74 in.
     * Every primary route point keeps its pre-v43 field position by adding it. */
    public static final double HIVE_LEFT_OF_PRIMARY_START_INCHES = PRIMARY_START_X_INCHES - RED_HIVE_X_INCHES;
    public static final double HIVE_LEFT_OF_PRIMARY_START_CM = HIVE_LEFT_OF_PRIMARY_START_INCHES * INCH_TO_CM;
    /** The launcher (and the Limelight above it) is this far LEFT of the robot's center, looking at
     * the hive: the team moved it so that the primary start above puts it on the hive's center.
     * Taken from that placement, not measured; aims point the LAUNCHER at the target. */
    public static final double LAUNCHER_LEFT_OF_CENTER_INCHES = HIVE_LEFT_OF_PRIMARY_START_INCHES;
    /** Robot-center X (red, inches) of a primary start whose midline-side edge is gapInches from the
     * center seam. The hive modes take the gap from their INIT row "Start gap to center seam in". */
    public static double primaryStartXInches(double gapInches) { return ALLIANCE_MIDLINE_X_INCHES - gapInches - ROBOT_HALF_INCHES; }
    /** Our cell's middle, inches LEFT of a primary start with that gap (2.74 at 1 in, 0 at 3.74 in). */
    public static double hiveLeftOfPrimaryStartInches(double gapInches) { return primaryStartXInches(gapInches) - RED_HIVE_X_INCHES; }
    public static final double PRIMARY_WALL_CLEARANCE_CM = 10;
    /** Primary leg 2 ends this far left of the HIVE CENTER (field X = hive - 25 in), clear of the frame. */
    public static final double PRIMARY_SIDE_CLEARANCE_INCHES = 25;
    public static final double SUPPORT_WALL_CLEARANCE_INCHES = 8;
    private static final double START_Z = END_WALL_INCHES - ROBOT_HALF_INCHES;
    /** Estimated middle of the hive cell's opening, this far from the field center toward the
     * shooter (docs/POLLEN_FIELD_ROUTES.md). From the wall start it is straight ahead, 45.9 in. */
    public static final double HIVE_OPENING_Z_INCHES = 15.7;
    /** EXPERIMENTAL angled shot: the team's ASSUMED shooting spot (strategy canvas, 2026-11-08),
     * as the robot's center: this far forward of the wall start and this far left of the HIVE
     * CENTER (v43: of the hive, not of the start, so the field spot is unchanged). */
    public static final double ANGLED_OFF_WALL_INCHES = 5, ANGLED_SIDE_INCHES = 30;
    private static final double LEFT_TO_PARK = PRIMARY_START_X_INCHES - PARK_X_INCHES;
    /** Primary leg 2 (Odo/Timed Shoot + Park): left from the start to hive - 25 in. */
    public static double primarySideLegInches() { return PRIMARY_SIDE_CLEARANCE_INCHES + HIVE_LEFT_OF_PRIMARY_START_INCHES; }
    /** Angled leg 2 (Calibration Angled Primary): left from the start to hive - ANGLED_SIDE_INCHES. */
    public static double angledSideLegInches() { return ANGLED_SIDE_INCHES + HIVE_LEFT_OF_PRIMARY_START_INCHES; }

    /** Hive opening straight ahead of the primary start, inches forward of the start (45.9 in). */
    public static double hiveOpeningForwardInches() { return START_Z - HIVE_OPENING_Z_INCHES; }
    /** Heading (degrees, +left) that points the LAUNCHER at the middle of our hive cell's opening,
     * for a robot whose center is this far from the PRIMARY START (away = off the wall, left =
     * toward the garden). From the start itself it is 0: the launcher is on the hive's center. */
    public static double aimDegrees(double awayInches, double leftInches) {
        return aimDegrees(awayInches, leftInches, 0);
    }
    /** Aim at a point targetLeftInches left of the opening's middle (negative = right of it). */
    public static double aimDegrees(double awayInches, double leftInches, double targetLeftInches) {
        return aimDegrees(awayInches, leftInches, targetLeftInches, HIVE_LEFT_OF_PRIMARY_START_INCHES);
    }
    /** General form: the cell's middle is cellLeftInches left of the start, and the launcher is
     * LAUNCHER_LEFT_OF_CENTER_INCHES left of the robot's center. The robot turns about its center,
     * so the heading puts the launcher's line (not the center's) through the target. */
    public static double aimDegrees(double awayInches, double leftInches, double targetLeftInches, double cellLeftInches) {
        double forward = hiveOpeningForwardInches() - awayInches;
        double left = cellLeftInches + targetLeftInches - leftInches; // target, from the robot's center
        double range = Math.hypot(forward, left);
        double offset = Math.asin(Math.max(-1, Math.min(1, LAUNCHER_LEFT_OF_CENTER_INCHES / range)));
        return Math.toDegrees(Math.atan2(left, forward) - offset);
    }
    /** Beta angled modes: where our cell's middle is, inches LEFT of the robot's start (looking at
     * the hive). Primary: 2.74 in (v43 start, 1 in from the seam). Secondary: 1 in gaps to the
     * flower and the seam (10.03 in from the seam): 2.71 in to its right. Blue mirrors red. */
    public static double cellLeftOfStartInches(boolean secondary) {
        return secondary ? RED_HIVE_X_INCHES - SUPPORT_START_X_INCHES : HIVE_LEFT_OF_PRIMARY_START_INCHES;
    }
    /** Launcher-to-cell distance (inches) from a robot center this far from the primary start. */
    public static double aimDistanceInches(double awayInches, double leftInches) {
        return Math.hypot(leftInches - HIVE_LEFT_OF_PRIMARY_START_INCHES, hiveOpeningForwardInches() - awayInches);
    }

    /** Primary leg 3: after forward wall clearance and the left hive clearance. */
    public static double primaryDistanceCm() {
        return Math.hypot(primaryLeftRemainingInches(), primaryForwardRemainingInches()) * INCH_TO_CM;
    }
    public static double primaryAngleDegrees() {
        return Math.toDegrees(Math.atan2(primaryLeftRemainingInches(), primaryForwardRemainingInches()));
    }
    private static double primaryLeftRemainingInches() {
        return LEFT_TO_PARK - primarySideLegInches();
    }
    private static double primaryForwardRemainingInches() {
        return START_Z - PRIMARY_WALL_CLEARANCE_CM / INCH_TO_CM - PRIMARY_PARK_Z_INCHES;
    }

    /** Angled shot: heading that points the robot at the middle of the hive opening from the
     * shooting spot, in degrees (+left). The spot is to the LEFT of the hive, so the answer is
     * negative (turn right): about -36 degrees, with the opening about 51 in away. */
    public static double angledAimDegrees() {
        return aimDegrees(ANGLED_OFF_WALL_INCHES, angledSideLegInches(), 0);
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
        return LEFT_TO_PARK - angledSideLegInches();
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
