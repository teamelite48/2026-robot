// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.controllers.angle.lib;


public interface AngleController {
    void init();
    double getCurrentAngle();
    double getTargetAngle();
    double getAbsoluteAngle();
    void setAngle(double angle);
    boolean isInitialized();
    double getRawCurrentAngle();
}
