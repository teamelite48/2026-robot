// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterFeed;

import com.ctre.phoenix6.CANBus;

import frc.robot.components.motors.lib.MotorConfig;

public class ShooterFeedConfig {
    public static final CANBus CANIVORE = new CANBus("canivore");

    public static final double FEED_SPEED = 0.3;

    public static final boolean BALL_SENSED_VALUE = false;
    public static final int BALL_COOLDOWN_TIMER = 100;
    public static final long TIME_TO_SHOOT_MILLIS = 250;

    public static MotorConfig getShooterFeedConfig() {

        var config = new MotorConfig(15, CANIVORE);

        // config.isInverted = true;
        config.isBrakeModeEnabled = true;
        config.currentLimit = 40;
        config.positionConversionFactor = (1.0 / 1.0);

        return config;
    }
}
