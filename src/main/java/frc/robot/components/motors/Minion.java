// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.motors;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.components.motors.lib.Motor;
import frc.robot.components.motors.lib.MotorConfig;

public class Minion implements Motor {

    final TalonFXS talon;
    final MotorConfig config;

    final DutyCycleOut dutyCycleOut;
    final PositionDutyCycle positionDutyCycle;

    public Minion(MotorConfig motorConfig) {

        this.config = motorConfig;

        dutyCycleOut = new DutyCycleOut(0.0).withEnableFOC(config.enableFOC);
        positionDutyCycle = new PositionDutyCycle(0).withSlot(0).withEnableFOC(config.enableFOC);

        if (motorConfig.canBus == null) {
            talon = new TalonFXS(motorConfig.canBusId);
        }
        else {
            talon = new TalonFXS(motorConfig.canBusId, motorConfig.canBus);
        }

        var talonConfig = new TalonFXSConfiguration();

        talonConfig.MotorOutput
            .withInverted(
                motorConfig.isInverted
                    ? InvertedValue.CounterClockwise_Positive
                    : InvertedValue.Clockwise_Positive
            );

        talonConfig.MotorOutput
            .withNeutralMode(
                motorConfig.isBrakeModeEnabled
                    ? NeutralModeValue.Brake
                    : NeutralModeValue.Coast
            );

        if (motorConfig.currentLimit != null) {
            // talonConfig.CurrentLimits
            //     .withSupplyCurrentLimitEnable(true)
            //     .withSupplyCurrentLimit(motorConfig.currentLimit);
            double currentAmps = motorConfig.currentLimit;
            talonConfig.CurrentLimits.StatorCurrentLimit = currentAmps;  // Amps
            talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        }

        if (motorConfig.pidParameters != null) {
            talonConfig.Slot0.kP = motorConfig.pidParameters.P;
            talonConfig.Slot0.kI = motorConfig.pidParameters.I;
            talonConfig.Slot0.kD = motorConfig.pidParameters.D;
        }

        talon.getConfigurator().apply(talonConfig);

        if (motorConfig.initialPosition != null) {
            talon.setPosition(
                calculateRotations(
                    motorConfig.initialPosition,
                    motorConfig.positionConversionFactor
                )
            );
        }
    }

    @Override
    public double getPosition() {
        return talon.getPosition().getValueAsDouble() * config.positionConversionFactor;
    }

    @Override
    public double getVelocity() {
        return talon.getVelocity().getValueAsDouble() * config.positionConversionFactor;
    }

    @Override
    public void setPosition(double position) {
        var rotations = calculateRotations(position, config.positionConversionFactor);

        talon.setControl(
            positionDutyCycle
                .withPosition(rotations)
                .withFeedForward(config.feedForward)
        );
    }

    @Override
    public void setSpeed(double speed) {
        talon.setControl(dutyCycleOut.withOutput(speed));
    }

    @Override
    public void setVoltage(double volts) {
        talon.setVoltage(volts);
    }

    @Override
    public void stop() {
        talon.stopMotor();
    }

    @Override
    public void setInitialPosition() {
        // does nothing
    }

    @Override
    public void setInitialPosition(double position) {
        talon.setPosition(calculateRotations(position, config.positionConversionFactor));
    }

    private static double calculateRotations(double units, double conversion) {
        return units / conversion;
    }
}
