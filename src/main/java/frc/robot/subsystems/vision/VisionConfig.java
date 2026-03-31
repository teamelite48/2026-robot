package frc.robot.subsystems.vision;

import frc.robot.components.limelights.lib.LimelightCameraConfig;

public class VisionConfig {
    public static final double TURRET_MOUNT_ANGLE_DEGREES = 20.0;
    public static final double TURRET_MOUNT_HEIGHT_INCHES = 17.25;

    // Measured to center of April Tag
    public static final double HUB_APRILTAG_HEIGHT_INCHES = 44.25;
    // public static final double LOAD_STATION_APRILTAG_HEIGHT_INCHES = 58.5;
    // public static final double BARGE_APRILTAG_HEIGHT_INCHES = 74.0;
    // public static final double PROCESSOR_APRILTAG_HEIGHT_INCHES = 51.125;

    public static final double HAS_TARGET_DEGREES_THRESHOLD = 1.5;
    public static final double HAS_TARGET_AREA_THRESHOLD = 0.00001;


    public static LimelightCameraConfig getTurretLimelightConfig() {
        var config = new LimelightCameraConfig("limelight-turret");     // Browser: 10.0.48.11:5801
        config.mountPitch = 20.0;
        config.mountHeightInches = 17.25;
        config.hasTargetDegreesThreshold = 1.5;
        return config;
    }

    public static LimelightCameraConfig getLeftLimelightConfig() {
        var config = new LimelightCameraConfig("limelight-left");       // Browser: 10.0.48.13:5801
        config.mountPitch = 20.0;
        config.mountHeightInches = 17.25;
        return config;
    }

    public static LimelightCameraConfig getRightLimelightConfig() {
        var config = new LimelightCameraConfig("limelight-right");      // Browser: 10.0.48.15:5801
        config.mountPitch = 20.0;
        config.mountHeightInches = 17.25;
        return config;
    }
}
