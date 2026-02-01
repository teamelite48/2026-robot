// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.controllers.angle.lib;

public class AngleControllerConfig {
    public int canBusId;
    public Integer absoluteEncoderCanBusId = null;
    public Double angleOffsetDegrees = null;

    public AngleControllerConfig(int canBusId) {
        this.canBusId = canBusId;
    }

    public AngleControllerConfig(int canBusId, int absoluteEncoderCanBusId) {
        this.canBusId = canBusId;
        this.absoluteEncoderCanBusId = absoluteEncoderCanBusId;
    }
}
