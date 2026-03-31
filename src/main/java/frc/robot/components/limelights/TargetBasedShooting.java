// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.limelights;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.components.limelights.lib.LimelightCamera;
import frc.robot.components.limelights.lib.LimelightCameraConfig;

public class TargetBasedShooting implements LimelightCamera {

    private final NetworkTable table;
    private final LimelightCameraConfig config;
    private final DoubleArrayEntry botposeEntry;

    // private VisionTarget target;
    private Optional<Alliance> alliance = Optional.empty();

    private boolean isTracking = false;
    private double feetFromTarget;
    private double distanceFromTargetFeet;

    public TargetBasedShooting(LimelightCameraConfig cameraConfig) {
        this.config = cameraConfig;

        this.table = NetworkTableInstance.getDefault().getTable(config.hostname);

        this.botposeEntry = table.getDoubleArrayTopic("camerapose_targetspace").getEntry(new double[6]);

        setLedMode(1);  // Disable Limelight LED
    }

    public double getFeetFromTarget(){
        return distanceFromTargetFeet;
    }

    public boolean hasTargetWithinParameters() {
        return hasTarget() && Math.abs(getXOffsetDegrees()) < config.hasTargetDegreesThreshold;
    }

    @Override
    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) == 1.0 ? true: false;
    }

    @Override
    public double getXOffsetDegrees() {
        return table.getEntry("tx").getDouble(0.0);
    }

    @Override
    public double getYOffsetDegrees() {
        // TODO: negative when target is below cross hair, positive when it's above
        return table.getEntry("ty").getDouble(0.0);
    }

    @Override
    public double getTargetArea() {
        return table.getEntry("ta").getDouble(0.0);
    }

    @Override
    public long getTargetId() {
        return table.getEntry("tid").getInteger(0);
    }

    @Override
    public Pose2d getBotPose() {
        // Do Nothing
        return null;
    }

    @Override
    public double[] getRawBotPose() {
        // Do Nothing
        return null;
    }

    @Override
    public double getLatency() {
        // Do Nothing
        return 0.0;
    }

    @Override
    public void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }

    @Override
    public void setLedMode(int mode) {
        table.getEntry("ledMode").setNumber(mode);
    }

    @Override
    public String getHostname() {
        return config.hostname;
    }

    @Override
    public void logPoseToDashboard(String key) {
        // Do Nothing
    }

    private void updateDistanceToTarget() {

        double angleDegrees = config.mountPitch + getYOffsetDegrees();
        double angleRadians = Math.toRadians(angleDegrees);

        double heightDifferenceInches = config.aprilTagHeightInches - config.mountHeightInches;

        if (Math.abs(Math.tan(angleRadians)) < 1e-6) {
            return;
        }

        double distanceInches = heightDifferenceInches / Math.tan(angleRadians);

        distanceFromTargetFeet = distanceInches / 12.0;
    }
}
