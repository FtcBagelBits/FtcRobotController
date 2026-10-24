package org.firstinspires.ftc.teamcode.pollen;

/** Nominal routes derived from official BIOBUZZ STEP v26-27.2 for an 18x18-inch
 * robot with a centered launcher. Support uses an offset, angled wall-contact pose.
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
    public static final double SUPPORT_START_X_INCHES = -10.35;
    // Horizontal center of the opening after the initially down cell tips upward,
    // estimated from the opposite upright cell's front-rib projection in the CAD.
    public static final double SUPPORT_AIM_Z_INCHES = -15.7;
    public static final double PRIMARY_CLEARANCE_INCHES = 25;
    public static final double SUPPORT_CLEARANCE_INCHES = 8;
    private static final double START_Z = END_WALL_INCHES - ROBOT_HALF_INCHES;
    private static final double LEFT_TO_PARK = RED_HIVE_X_INCHES - PARK_X_INCHES;

    public static double primaryDistanceCm() {
        return Math.hypot(LEFT_TO_PARK - PRIMARY_CLEARANCE_INCHES,
                START_Z - PRIMARY_PARK_Z_INCHES) * INCH_TO_CM;
    }
    public static double primaryAngleDegrees() {
        return Math.toDegrees(Math.atan2(LEFT_TO_PARK - PRIMARY_CLEARANCE_INCHES,
                START_Z - PRIMARY_PARK_Z_INCHES));
    }
    public static double supportDistanceCm() {
        return Math.hypot(PARK_X_INCHES - supportClearanceXInches(),
                SUPPORT_PARK_Z_INCHES - supportClearanceZInches()) * INCH_TO_CM;
    }
    public static double supportAngleDegrees() {
        double yaw = supportYawRadians();
        double dx = PARK_X_INCHES - supportClearanceXInches();
        double dz = SUPPORT_PARK_Z_INCHES - supportClearanceZInches();
        double forward = -dx * Math.sin(yaw) + dz * Math.cos(yaw);
        double right = -dx * Math.cos(yaw) - dz * Math.sin(yaw);
        return -Math.toDegrees(Math.atan2(right, forward));
    }
    public static double supportStartZInches() {
        double yaw = supportYawRadians();
        return -END_WALL_INCHES + ROBOT_HALF_INCHES * (Math.cos(yaw) + Math.sin(yaw));
    }
    public static double supportClearanceXInches() {
        return SUPPORT_START_X_INCHES - SUPPORT_CLEARANCE_INCHES * Math.sin(supportYawRadians());
    }
    public static double supportClearanceZInches() {
        return supportStartZInches() + SUPPORT_CLEARANCE_INCHES * Math.cos(supportYawRadians());
    }
    public static double supportYawRadians() {
        // The rear corner touches the wall; rotation changes the center-to-wall offset.
        double yaw = 0;
        for (int i = 0; i < 20; i++) {
            double z = -END_WALL_INCHES + ROBOT_HALF_INCHES * (Math.cos(yaw) + Math.sin(yaw));
            yaw = Math.atan2(SUPPORT_START_X_INCHES - RED_HIVE_X_INCHES, SUPPORT_AIM_Z_INCHES - z);
        }
        return yaw;
    }
}
