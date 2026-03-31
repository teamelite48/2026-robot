// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.limelights.lib;

import edu.wpi.first.math.geometry.Pose2d;

public interface LimelightCamera {
    public boolean hasTarget();
    public double getXOffsetDegrees();
    public double getYOffsetDegrees();
    public double getTargetArea();
    public long getTargetId();
    public Pose2d getBotPose();     // Field-space pose (MegaTag)
    public double[] getRawBotPose();
    public double getLatency();
    public void setPipeline(int pipeline);
    public void setLedMode(int mode);
    public void logPoseToDashboard(String key);
    public String getHostname();
}
