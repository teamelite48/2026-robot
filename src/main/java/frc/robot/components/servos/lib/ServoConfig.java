// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.servos.lib;

import com.ctre.phoenix6.CANBus;

public class ServoConfig {
    public int id;               // PWM Port (0-5 on Hub, or 0-9 on Rio)
    public int hubId = -1;       // CAN ID of the REV Hub (ignore if Rio)
    public CANBus canBus = null;     // null Rio PWM, name for CANivore (e.g., "canivore")

    // Degrees
    public double minAngle = 0;
    public double maxAngle = 180;

    // PWM Timing (microseconds)
    public Integer minPulseMs = null;
    public Integer maxPulseMs = null;


    /** For Rio PWM */
    public ServoConfig(int pwmChannel) {
        this.id = pwmChannel;
    }

    /** For CAN-based Hubs */
    public ServoConfig(int canId, int port) {
        this.hubId = canId;
        this.id = port;
    }

    public ServoConfig(int canId, int port, CANBus canBus) {
        this.hubId = canId;
        this.id = port;
        this.canBus = canBus;
    }

    public ServoConfig withLimits(double min, double max) {
        this.minAngle = min;
        this.maxAngle = max;
        return this;
    }

    public ServoConfig withPulseBounds(int minMs, int maxMs) {
        this.minPulseMs = minMs;
        this.maxPulseMs = maxMs;
        return this;
    }
}
