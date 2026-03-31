// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.cameras.lib;

import edu.wpi.first.cscore.VideoSource;

public interface Camera {
    public String getName();
    public boolean isConnected();
    public void setFPS(int fps);
    public void setResolution(int width, int height);
    public VideoSource getVideoSource();
}
