// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.servos;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;

import frc.robot.components.servos.lib.BaseServo;
import frc.robot.components.servos.lib.ServoConfig;

public class CanRevServo implements BaseServo {
    private final ServoConfig config;
    private final ServoHub hub;
    private final ServoChannel servo;

    private double currentPosition = 0;
    private boolean isServoEnabled = false;

    public CanRevServo(ServoConfig config) {
        this.config = config;
        hub = new ServoHub(config.hubId);

        servo = hub.getServoChannel(fromInt(config.id));

        if (config.maxPulseMs != null) {
            servo.setPulseWidth(config.maxPulseMs);
        }

        // Ensure the channel is active
        resume();
    }

    @Override
    public double getAngle() {
        return currentPosition * (config.maxAngle - config.minAngle) + config.minAngle;
    }

    @Override
    public double getPosition() {
        return currentPosition;
    }

    @Override
    public void setAngle(double degrees) {
        // Map degrees to 0.0 - 1.0 position
        double position = (degrees - config.minAngle) / (config.maxAngle - config.minAngle);
        setPosition(position);
    }

    @Override
    public void setPosition(double position) {
        this.currentPosition = Math.max(0, Math.min(1, position));

        // Map 0.0-1.0 to the configured pulse width (microseconds)
        int minPulse = (config.minPulseMs != null) ? config.minPulseMs : 500;   // 0.5ms
        int maxPulse = (config.maxPulseMs != null) ? config.maxPulseMs : 2500;  // 2.5ms

        int pulseWidth = (int) (minPulse + (currentPosition * (maxPulse - minPulse)));

        servo.setPulseWidth(pulseWidth);
    }

    @Override
    public void stop() {
        // Disabling stops the PWM signal; setPowered(false) cuts the line
        servo.setEnabled(false);
        servo.setPowered(false);
        isServoEnabled = false;
    }

    // If call stop() need to re-enable if you plan to move it again after a stop
    public void resume() {
        servo.setEnabled(true);
        servo.setPowered(true);
        isServoEnabled = true;
    }

    public double getCurrent() {
        return servo.getCurrent();
    }

    public boolean getIsServoEnabled() {
        return isServoEnabled;
    }

    private static ChannelId fromInt(int id) {
        return switch (id) {
            case 0 -> ChannelId.kChannelId0;
            case 1 -> ChannelId.kChannelId1;
            case 2 -> ChannelId.kChannelId2;
            case 3 -> ChannelId.kChannelId3;
            case 4 -> ChannelId.kChannelId4;
            case 5 -> ChannelId.kChannelId5;
            default -> ChannelId.kChannelId0;
        };
    }
}
