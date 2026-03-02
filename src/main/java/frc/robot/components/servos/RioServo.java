// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.servos;
import edu.wpi.first.wpilibj.Servo;

import frc.robot.components.servos.lib.BaseServo;
import frc.robot.components.servos.lib.ServoConfig;

public class RioServo implements BaseServo {
    private final Servo servo;
    private final ServoConfig config;

    public RioServo(ServoConfig config) {
        this.config = config;
        servo = new Servo(config.id);

        if (config.minPulseMs != null && config.maxPulseMs != null) {
            // High-end FRC servos often need their PWM bounds tuned
            servo.setBoundsMicroseconds(config.maxPulseMs, 0, 0, 0, config.minPulseMs);
        }
    }

    @Override
    public double getAngle() {
        return servo.getAngle();
    }

    @Override
    public double getPosition() {
        return servo.getPosition();
    }

    @Override
    public void setAngle(double degrees) {
        // Clamp the value to protect the hardware
        double clamped = Math.max(config.minAngle, Math.min(config.maxAngle, degrees));
        servo.setAngle(clamped);
    }

    @Override
    public void setPosition(double position) {
        // 0.0 to 1.0 scale
        servo.set(position);
    }

    @Override
    public void stop() {
        servo.set(0.0);
    }
}
