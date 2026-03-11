// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.HashMap;

import frc.robot.RobotConfig;
import frc.robot.components.motors.lib.MotorConfig;
import frc.robot.lib.PIDParameters;

public class ShooterConfig {

    public enum ShooterPreset {
        LOW,
        MEDIUM,
        HIGH
    }

    public static final double LOW_RPM = 1000.0;
    public static final double MEDIUM_RPM = 3000.0;
    public static final double HIGH_RPM = 5000.0;

    public static final double ON_SPEED_TOLERANCE_RPM = 100.0;

    public static final double IDLE_RPM = 550.0;

    public static final double RPM_BUMP = 100.0;

    public static final HashMap<Integer, Double> FEET_TO_RPM_MAP = new HashMap<Integer, Double>() {{
        put(0, 1100.0);
        put(1, 1100.0);
        put(2, 1100.0);
        put(3, 1100.0);
        put(4, 1100.0);
        put(5, 1100.0);
        put(6, 1100.0);
        put(7, 1700.0);
        put(8, 1800.0);
        put(9, 1900.0);
        put(10, 2000.0);
        put(11, 2050.0);
        put(12, 2100.0);
        put(13, 2175.0);
        put(14, 2250.0);
        put(15, 2350.0);
        put(16, 2450.0);
        put(17, 2550.0);
        put(18, 2850.0);
        put(19, 3500.0);
        put(20, 4100.0);
        put(21, 4400.0);
        put(22, 4750.0);
    }};

    public static MotorConfig getShooterRightConfig() {

        var config = new MotorConfig(19, RobotConfig.CANIVORE_48);

        config.isInverted = false;
        config.isBrakeModeEnabled = false;
        config.currentLimit = 40;
        config.positionConversionFactor = (1.0 / 1.0);
        config.pidParameters = new PIDParameters(0.00003, 0.0, 0.0, 0.0);

        return config;
    }
    public static MotorConfig getShooterLeftConfig() {

        var config = new MotorConfig(18, RobotConfig.CANIVORE_48);

        config.isInverted = true;
        config.isBrakeModeEnabled = false;
        config.currentLimit = 40;
        config.positionConversionFactor = (1.0 / 1.0);
        config.pidParameters = new PIDParameters(0.00003, 0.0, 0.0, 0.0);
        config.enableFOC = false;

        return config;
    }
}
