// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.cameras.lib;

public class CameraConfig {
    public String name;
    public int portId; // The USB port index (usually 0 or 1)
    public int width = 160;
    public int height = 120;
    public int framesPerSecond = 20;
    public int exposure = -1; // -1 for Auto, 0-100 for manual

    public CameraConfig(String name, int portId) {
        this.name = name;
        this.portId = portId;
    }
}