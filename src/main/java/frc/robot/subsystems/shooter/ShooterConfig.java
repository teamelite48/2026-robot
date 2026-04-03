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
        HIGH,
        PASS
    }

    public static final double LOW_RPM = 2000.0;
    public static final double MEDIUM_RPM = 2850.0;
    public static final double HIGH_RPM = 3300.0;
    public static final double PASS_RPM = 3800.0;

    public static final double ON_SPEED_TOLERANCE_RPM = 200.0;

    public static final double IDLE_RPM = 750.0;

    public static final double RPM_BUMP = 100.0;

    // If 3000 RPM is your 'Sweet Spot' and we assume 70% efficiency:
    // (3000 * 0.319 / 60) * 0.70 = ~11.1 m/s
    public static final double AVERAGE_FUEL_VELOCITY = 10.0;

    public static final HashMap<Integer, Double> FEET_TO_RPM_MAP = new HashMap<Integer, Double>() {{
        put(2, 2400.0);
        put(3, 2400.0);
        put(4, 2400.0);
        put(5, 2400.0);
        put(6, 2500.0);
        put(7, 2600.0);
        put(8, 2700.0);
        put(9, 2800.0);
        put(10, 2900.0);
        put(11, 3000.0);
        put(12, 3100.0);
        put(13, 3200.0);
        put(14, 3300.0);
        put(15, 3600.0);
        put(16, 3700.0);
        put(17, 3800.0);
        put(18, 3900.0);
        put(19, 4000.0);
        put(20, 4100.0);
        put(21, 4200.0);
        put(22, 4300.0);
        put(23, 4400.0);
        put(24, 4500.0);
        put(54, 4500.0);
    }};

    public static MotorConfig getShooterRightConfig() {

        var config = new MotorConfig(19, RobotConfig.CANIVORE_48);

        config.isInverted = false;
        config.isBrakeModeEnabled = false;
        config.supplyCurrentLimit = 40;
        config.statorCurrentLimit = 60;
        config.positionConversionFactor = (1.0 / 1.0);
        config.pidParameters = new PIDParameters(0.03, 0.0, 0.0, 0.0, 0.125);

        return config;
    }
    public static MotorConfig getShooterLeftConfig() {

        var config = new MotorConfig(18, RobotConfig.CANIVORE_48);

        config.isInverted = true;
        config.isBrakeModeEnabled = false;
        config.supplyCurrentLimit = 40;
        config.statorCurrentLimit = 60;
        config.positionConversionFactor = (1.0 / 1.0);
        config.pidParameters = new PIDParameters(0.03, 0.0, 0.0, 0.0, 0.125);
        config.enableFOC = false;

        return config;
    }
}
