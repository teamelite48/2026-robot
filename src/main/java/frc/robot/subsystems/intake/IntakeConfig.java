// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.RobotConfig;
import frc.robot.components.motors.lib.MotorConfig;

public class IntakeConfig {

    public static final double INTAKE_SPEED = -0.25;

    public static MotorConfig getIntakeRightConfig() {

        var config = new MotorConfig(11, RobotConfig.CANIVORE_48);

        config.isInverted = true;
        config.isBrakeModeEnabled = false;

        return config;
    }
}
