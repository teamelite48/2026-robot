package frc.robot.components.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.components.motors.lib.Motor;
import frc.robot.components.motors.lib.MotorConfig;

public class Kraken implements Motor {

    final TalonFX talonFx;
    final MotorConfig config;

    final PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0).withSlot(0);

    public Kraken(MotorConfig motorConfig) {

        this.config = motorConfig;

        if (motorConfig.canivoreBus == null) {
            talonFx = new TalonFX(motorConfig.canBusId);
        }
        else {
            talonFx = new TalonFX(motorConfig.canBusId, motorConfig.canivoreBus);
        }

        var talonFxConfig = new TalonFXConfiguration();

        talonFxConfig.MotorOutput
            .withInverted(motorConfig.isInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive);

        talonFxConfig.MotorOutput
            .withNeutralMode(motorConfig.isBrakeModeEnabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);


        if (motorConfig.maxForwardSpeed != null) {
            talonFxConfig.MotorOutput
                .withPeakForwardDutyCycle(motorConfig.maxForwardSpeed);
        }

        if (motorConfig.maxReverseSpeed != null) {
            talonFxConfig.MotorOutput
                .withPeakReverseDutyCycle(motorConfig.maxReverseSpeed);
        }

        if (motorConfig.currentLimit != null) {
            talonFxConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(motorConfig.currentLimit);
        }

        talonFxConfig.Feedback
            .withSensorToMechanismRatio(1.0)
            .withRotorToSensorRatio(1.0);

        if (motorConfig.forwardLimit != null) {
            talonFxConfig.SoftwareLimitSwitch
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(calculateRotations(motorConfig.forwardLimit, motorConfig.positionConversionFactor));
        }

        if (motorConfig.reverseLimit != null) {
            talonFxConfig.SoftwareLimitSwitch
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(calculateRotations(motorConfig.reverseLimit, motorConfig.positionConversionFactor));
        }

        if (motorConfig.pidParameters != null) {
            talonFxConfig.Slot0.kP = motorConfig.pidParameters.P;
            talonFxConfig.Slot0.kI = motorConfig.pidParameters.I;
            talonFxConfig.Slot0.kD = motorConfig.pidParameters.D;
        }

        talonFx.getConfigurator().apply(talonFxConfig);

        if (motorConfig.initialPosition != null) {
            talonFx.setPosition(calculateRotations(motorConfig.initialPosition, motorConfig.positionConversionFactor));
        }
    }

    public double getPosition() {
        return talonFx.getPosition().getValueAsDouble() * config.positionConversionFactor;
    }

    public double getVelocity() {
        return talonFx.getVelocity().getValueAsDouble() * config.positionConversionFactor;
    }

    public void setPosition(double position) {
        var rotations = calculateRotations(position, config.positionConversionFactor);
        talonFx.setControl(positionDutyCycle.withPosition(rotations).withFeedForward(config.feedFoward));
    }

    public void setSpeed(double speed) {
        talonFx.set(speed);
    }

    public void setVoltage(double volts) {
        talonFx.setVoltage(volts);
    }

    public void stop() {
        talonFx.stopMotor();
    }

    private static double calculateRotations(double units, double positionConversionFactor) {
        return  units / positionConversionFactor;
    }

    @Override
    public void setInitialPosition() {
        // does nothing
    }

    @Override
    public void setInitialPosition(double position) {
        // does nothing
    }
}
