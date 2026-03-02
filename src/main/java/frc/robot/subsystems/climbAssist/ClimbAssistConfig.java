// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climbAssist;

import frc.robot.components.servos.lib.ServoConfig;

public class ClimbAssistConfig {
    public static final double LOCK_VALUE = 1.0;
    public static final double UNLOCK_VALUE = 0.0;

    public static ServoConfig getServoConfig() {
        var config = new ServoConfig(6, 0)
            // .withLimits(0, 90)
        ;
        return config;
    }
}
