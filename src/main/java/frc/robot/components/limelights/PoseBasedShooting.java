// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.limelights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.components.limelights.lib.LimelightCamera;
import frc.robot.components.limelights.lib.LimelightCameraConfig;
import frc.robot.lib.LimelightHelpers;

public class PoseBasedShooting implements LimelightCamera {

    private final NetworkTable table;
    private final LimelightCameraConfig config;
    private final DoubleArrayEntry botposeEntry;

    public PoseBasedShooting(LimelightCameraConfig cameraConfig) {
        this.config = cameraConfig;
        this.table = NetworkTableInstance.getDefault().getTable(config.hostname);

        // Limelight MegaTag2 array: [x, y, z, roll, pitch, yaw, latency, tagCount, tagSpan, avgTagDist, avgTagArea]
        this.botposeEntry = table.getDoubleArrayTopic("botpose_wpiblue").getEntry(new double[7]);

        setLedMode(1);  // Disable Limelight LED
    }

    public void updateMegaTag2(double gyroYawDegrees) {
        LimelightHelpers.SetRobotOrientation(config.hostname, gyroYawDegrees, 0, 0, 0, 0, 0);
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

    public double getFeetFromTarget() {

        Pose2d currentPose = getBotPose();
        // Distance formula: $\sqrt{(x_2-x_1)^2 + (y_2-y_1)^2}$
        double distanceMeters = currentPose.getTranslation().getDistance(config.targetTranslation);
        return distanceMeters * 3.28084; // Convert meters to feet
    }

    @Override
    public Pose2d getBotPose() {

        double[] poseArray = botposeEntry.get();
        if (poseArray.length < 7) return new Pose2d();

        // Limelight botpose array: [x, y, z, roll, pitch, yaw, latency]
        return new Pose2d(
            new Translation2d(poseArray[0], poseArray[1]),
            Rotation2d.fromDegrees(poseArray[5])
        );
    }

    @Override
    public double[] getRawBotPose() {
        return botposeEntry.get();
    }

    @Override
    public double getLatency() {
        double[] poseArray = botposeEntry.get();
        return (poseArray.length > 6) ? poseArray[6] : 0.0;
    }

    @Override
    public void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }

    @Override
    public void setLedMode(int mode) {
        // Note Limelight 4 doesn't have LEDs
        table.getEntry("ledMode").setNumber(mode);
    }

    @Override
    public void logPoseToDashboard(String key) {
        // TODO: add Elastic stuff to display botpose
    }

    @Override
    public String getHostname() {
        return config.hostname;
    }
}
