package frc.robot.subsystems.drive.lib;

public class SwerveMath {

    public final static double PI = Math.PI;
    public final static double TAU = 2.0 * PI;

    public static double normalizeAngle(double radians) {

        radians %= TAU;

        if (radians < 0.0) {
            radians += TAU;
        }

        return radians;
    }

    public static double toRadians(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double toDegrees(double radians) {
        return Math.toDegrees(radians);
    }
}
