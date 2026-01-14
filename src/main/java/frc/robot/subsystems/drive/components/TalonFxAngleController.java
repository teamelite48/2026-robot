package frc.robot.subsystems.drive.components;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Units;
import edu.wpi.first.units.measure.Angle;

import static frc.robot.subsystems.drive.DriveConfig.*;
import static frc.robot.subsystems.drive.lib.SwerveMath.*;

public class TalonFxAngleController {

    private final TalonFX motorController;
    private final CANcoder absoluteEncoder;

    private double targetAngle = 0.0;
    private int resetIteration = 0;

    public TalonFxAngleController(int motorId, int absoluteEncoderId, double offsetDegrees) {

        absoluteEncoder = new CANcoder(absoluteEncoderId);

        var canCoderConfig = new CANcoderConfiguration();

        canCoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
        canCoderConfig.MagnetSensor.withMagnetOffset(-(offsetDegrees / 360));
        canCoderConfig.MagnetSensor.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        absoluteEncoder.getConfigurator().apply(canCoderConfig);

        motorController = new TalonFX(motorId);

        var talonFxConfig = new TalonFXConfiguration();

        talonFxConfig.MotorOutput
            // .withInverted(ANGLE_MOTOR_INVERTED)
            .withNeutralMode(NeutralModeValue.Brake);

        talonFxConfig.Feedback
            .withSensorToMechanismRatio(ANGLE_POSITION_TO_RADIANS_CONVERSION_FACTOR)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
            .withFeedbackRotorOffset(getAbsoluteAngle());

        talonFxConfig.CurrentLimits
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(ANGLE_MOTOR_CURRENT_LIMIT);

        var slot0Configs = talonFxConfig.Slot0;

        slot0Configs.kP = 0.11;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.1;

        motorController.getConfigurator().apply(new TalonFXConfiguration());
        motorController.getConfigurator().apply(talonFxConfig, 0.05);
    }

    public void setAngle(double desiredAngle) {

        double currentAngle = motorController.getPosition().getValueAsDouble();

        // Reset the NEO's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter anymore.
        if (motorController.getVelocity().getValueAsDouble() < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                currentAngle = getAbsoluteAngle();
                motorController.setPosition(currentAngle);

                resetIteration = 0;
            }
        }
        else {
            resetIteration = 0;
        }

        var currentAngleMod = normalizeAngle(currentAngle);

        // The target angle has the range [0, 2pi) but the Neo's encoder can go above that
        double adjustedDesiredAngle = desiredAngle + currentAngle - currentAngleMod;

        if (desiredAngle - currentAngleMod > PI) {
            adjustedDesiredAngle -= TAU;
        }
        else if (desiredAngle - currentAngleMod < -PI) {
            adjustedDesiredAngle += TAU;
        }

        this.targetAngle = adjustedDesiredAngle;

        motorController.setPosition(Angle.ofBaseUnits(this.targetAngle, Units.Radians));
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getCurrentAngle() {
        return normalizeAngle(motorController.getPosition().getValueAsDouble());
    }

    public double getAbsoluteAngle() {

        return normalizeAngle(absoluteEncoder.getAbsolutePosition().getValueAsDouble() * TAU);
    }
}