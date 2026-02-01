// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.controllers.drive.lib;


public interface DriveController {
    double getTargetVelocity();
    double getCurrentVelocity();
    double getCurrentPosition();
    int getCanBusId();
    void setVelocity(double metersPerSecond);
}
