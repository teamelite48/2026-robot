// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.limelights.lib;

public class LimelightCameraConfig {
    public String hostname = null;  // Find this in the Limelight browser (Example: 10.0.48.11:5801)
    public Double mountPitch = null;  // Previously the "mount_angle_degrees"
    public Double mountRoll = null;
    public Double mountYaw = null;
    public double mountHeightInches;
    public double mountOffsetForwardFromCenterInches;
    public double mountOffsetLeftFromCenterInches;
    public double hasTargetDegreesThreshold = 0.0;
    public Double aprilTagHeightInches = null;  // TODO: Target based shooting needs this to be any # of april tags

    public LimelightCameraConfig(String host) {
        this.hostname = host;
    }
}
