// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;

import frc.robot.components.motors.lib.MotorConfig;
import frc.robot.lib.PIDParameters;

public class IntakeConfig {

    public static final CANBus CANIVORE = new CANBus("canivore");

    public static final double EXTEND_SPEED = 0.25;
    public static final double RETRACT_SPEED = -0.3;
    public static final double INTAKE_SPEED = -0.25;
    public static final double EXTEND_LIMIT = 513.52;
    public static final double MID_POSITION = 341.67;
    public static final double RETRACT_LIMIT = 0.0;
    public static final double HOME_POSITION = RETRACT_LIMIT + 2.0;

    public static MotorConfig getIntakeRightConfig() {

        var config = new MotorConfig(11, CANIVORE);
        // var config = new MotorConfig(11);

        config.isInverted = true;
        config.isBrakeModeEnabled = true;

        return config;
    }

     public static MotorConfig getIntakeDeployConfig() {

        var config = new MotorConfig(13);

        config.isInverted = false;
        config.forwardLimit = EXTEND_LIMIT;
        config.reverseLimit = HOME_POSITION;
        config.positionConversionFactor = (30.0 / 16.0) * 9.0 * (54.0 / 28.0); //32.545;
        config.initialPosition = HOME_POSITION;
        config.pidParameters = new PIDParameters(0.9, 0.0, 0.0); //PIDParameters(0.6, 0.0, 0.0);
        config.isBrakeModeEnabled = true;

        return config;
    }
}
