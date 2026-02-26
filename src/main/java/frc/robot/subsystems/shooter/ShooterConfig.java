// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.HashMap;

import com.ctre.phoenix6.CANBus;

import frc.robot.components.motors.lib.MotorConfig;
import frc.robot.lib.PIDParameters;

public class ShooterConfig {

    public static final CANBus CANIVORE = new CANBus("canivore");

    public static final double LOW_RPM = 0.5;
    public static final double MEDIUM_RPM = 1.0;
    public static final double RPM_BUMP = 1.0;

    public static final HashMap<Integer, Integer> DISTANCE_TO_RPM_MAP = new HashMap<Integer, Integer>() {{
        put(0, 1100);
        put(1, 1100);
        put(2, 1100);
        put(3, 1100);
        put(4, 1100);
        put(5, 1100);
        put(6, 1100);
        put(7, 1700);
        put(8, 1800);
        put(9, 1900);
        put(10, 2000);
        put(11, 2050);
        put(12, 2100);
        put(13, 2175);
        put(14, 2250);
        put(15, 2350);
        put(16, 2450);
        put(17, 2550);
        put(18, 2850);
        put(19, 3500);
        put(20, 4100);
        put(21, 4400);
        put(22, 4750);
    }};

    public static MotorConfig getShooterRightConfig() {

        var config = new MotorConfig(19, CANIVORE);

        config.isInverted = true;
        config.isBrakeModeEnabled = true;
        config.pidParameters = new PIDParameters(0.00003, 0.0, 0.0);

        return config;
    }
    public static MotorConfig getShooterLeftConfig() {

        var config = new MotorConfig(18, CANIVORE);

        config.isInverted = false;
        config.isBrakeModeEnabled = true;
        config.pidParameters = new PIDParameters(0.00003, 0.0, 0.0);

        return config;
    }
}
