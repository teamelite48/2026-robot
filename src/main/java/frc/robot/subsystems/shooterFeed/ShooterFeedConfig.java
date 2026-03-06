// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterFeed;

import frc.robot.components.motors.lib.MotorConfig;

public class ShooterFeedConfig {
    public static final double FEED_SPEED = 0.5;

    public static final boolean BALL_SENSED = false;
    //public static final int BALL_COOLDOWN_TIMER = 100;
    public static final long TIME_TO_SHOOT_MILLIS = 250;

    public static MotorConfig getShooterFeedConfig() {

        var config = new MotorConfig(15);

        // config.isInverted = true;
        config.isBrakeModeEnabled = true;
        config.currentLimit = 40;

        return config;
    }
}
