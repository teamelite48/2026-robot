// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

import frc.robot.RobotConfig;
import frc.robot.components.motors.lib.MotorConfig;

public class SpindexerConfig {

    public static final double FEED_SPEED = 0.6;  //0.32 - feed-to-spindexer speed ratio = 1.4

    public static final double REVERSE_SPEED = -FEED_SPEED;

    public static MotorConfig getSpindexerConfig() {

        var config = new MotorConfig(14, RobotConfig.CANIVORE_48);

        // config.isInverted = true;
        config.isBrakeModeEnabled = false;
        config.supplyCurrentLimit = 30; // 40
        config.statorCurrentLimit = 40;
        config.positionConversionFactor = (1.0 / 3.0);
        config.enableFOC = false;

        return config;
    }
}
