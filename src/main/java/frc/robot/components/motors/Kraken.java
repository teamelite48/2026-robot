package frc.robot.components.motors;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.components.motors.lib.Motor;
import frc.robot.components.motors.lib.MotorConfig;

public class Kraken implements Motor {

    final TalonFX talonFx;
    final MotorConfig config;

    final DutyCycleOut dutyCycleOut;
    final PositionDutyCycle positionDutyCycle;

    private final StatusSignal<Angle> rotorPos;
    private final StatusSignal<AngularVelocity> rotorVel;

    public Kraken(MotorConfig motorConfig) {

        this.config = motorConfig;

        positionDutyCycle = new PositionDutyCycle(0).withSlot(0).withEnableFOC(config.enableFOC);
        dutyCycleOut = new DutyCycleOut(0.0).withEnableFOC(config.enableFOC);

        if (motorConfig.canBus == null) {
            talonFx = new TalonFX(motorConfig.canBusId);
        }
        else {
            talonFx = new TalonFX(motorConfig.canBusId, motorConfig.canBus);
        }

        var talonFxConfig = new TalonFXConfiguration();

        talonFxConfig.ClosedLoopGeneral.ContinuousWrap = config.continuousWrap;

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

        rotorPos = talonFx.getPosition();
        rotorVel = talonFx.getVelocity();
    }

    public double getPosition() {
        // return talonFx.getPosition().getValueAsDouble() * config.positionConversionFactor;
        return rotorPos.refresh().getValueAsDouble() * config.positionConversionFactor;
    }

    public double getVelocity() {
        // return talonFx.getVelocity().getValueAsDouble() * config.positionConversionFactor;
        return rotorVel.refresh().getValueAsDouble() * config.positionConversionFactor;
    }

    // public int getCanId() {
    //     return talonFx.getDeviceID();
    // }

    public void setPosition(double position) {
        var rotations = calculateRotations(position, config.positionConversionFactor);

        // If ContinuousWrap is true, normalize the target to [0, 1)
        // to prevent the internal accumulator from growing infinitely.
        if (config.continuousWrap) {
            rotations = rotations % 1.0;

            if (rotations < 0) {
                rotations += 1.0;
            }
        }

        talonFx.setControl(positionDutyCycle.withPosition(rotations).withFeedForward(config.feedForward));
    }

    public void setSpeed(double speed) {
        // talonFx.set(speed);  // This way ignores FOC config
        talonFx.setControl(dutyCycleOut.withOutput(speed));
    }

    public void setVoltage(double volts) {
        talonFx.setVoltage(volts);
    }

    public void stop() {
        talonFx.stopMotor();
    }

    // public void setFollower(TalonFX followerMotor) {
    //     followerMotor.setControl(new Follower(talonFx.getDeviceID(), MotorAlignmentValue.Opposed));
    // }

    @Override
    public void setInitialPosition() {
        // does nothing
    }

    @Override
    public void setInitialPosition(double position) {
        talonFx.setPosition(calculateRotations(position, config.positionConversionFactor));
    }

    private static double calculateRotations(double units, double positionConversionFactor) {
        return  units / positionConversionFactor;
    }
}
