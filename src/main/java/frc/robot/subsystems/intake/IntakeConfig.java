// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;

import frc.robot.components.motors.lib.MotorConfig;

public class IntakeConfig {

    public static final CANBus CANIVORE = new CANBus("canivore");

    public static final double INTAKE_SPEED = -0.25;

    public static MotorConfig getIntakeRightConfig() {

        var config = new MotorConfig(11, CANIVORE);

        config.isInverted = true;
        config.isBrakeModeEnabled = true;

        return config;
    }
}
