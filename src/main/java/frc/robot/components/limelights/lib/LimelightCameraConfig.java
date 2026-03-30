// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.limelights.lib;

public class LimelightCameraConfig {
    public String hostname = null;  // Find this in the Limelight browser (Example: 10.0.48.11:5801)
    public Double mount_pitch = null;  // Previously the "mount_angle_degrees"
    public Double mount_roll = null;
    public Double mount_yaw = null;
    public double mount_height_inches;
    public double mount_offset_forward_from_center_inches;
    public double mount_offset_left_from_center_inches;
    public double has_target_degrees_threshold = 0.0;
    public Double april_tag_height_inches = null;  // TODO: Target based shooting needs this to be any # of april tags

    public LimelightCameraConfig(String host) {
        this.hostname = host;
    }
}
