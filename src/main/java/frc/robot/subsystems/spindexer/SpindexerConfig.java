// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.CANBus;
import frc.robot.components.motors.lib.MotorConfig;

public class SpindexerConfig {

    public static final CANBus CANIVORE = new CANBus("canivore");

    public static final double FEED_SPEED = 0.5;
    public static final double REVERSE_SPEED = -FEED_SPEED;

    public static MotorConfig getSpindexerConfig() {

        var config = new MotorConfig(14, CANIVORE);

        // config.isInverted = true;
        config.isBrakeModeEnabled = true;
        config.currentLimit = 40;

        return config;
    }
}
