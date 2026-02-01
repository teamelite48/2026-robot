package frc.robot.components.controllers.angle;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.components.controllers.angle.lib.AngleController;
import frc.robot.components.controllers.angle.lib.AngleControllerConfig;
import frc.robot.components.swerve.lib.SwerveConfig;

import static frc.robot.components.swerve.lib.SwerveMath.*;


public class TalonFxAngleController implements AngleController {

    private final TalonFX motorController;
    private final CANcoder absoluteEncoder;
    private final SwerveConfig swerveConfig;
    private final AngleControllerConfig angleConfig;

    private double targetAngle = 0.0;
    private int resetIteration = 0;
    private boolean isInitialized = false;

    public TalonFxAngleController(SwerveConfig swerveConfig, AngleControllerConfig angleControllerConfig) {
        this.swerveConfig = swerveConfig;
        this.angleConfig = angleControllerConfig;
        this.absoluteEncoder = new CANcoder(angleConfig.absoluteEncoderCanBusId);

        var canCoderConfig = new CANcoderConfiguration();

        canCoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
        canCoderConfig.MagnetSensor.withMagnetOffset(-(angleConfig.angleOffsetDegrees / 360));
        canCoderConfig.MagnetSensor.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        absoluteEncoder.getConfigurator().apply(canCoderConfig);

        motorController = new TalonFX(angleConfig.canBusId);

        var talonFxConfig = new TalonFXConfiguration();

        talonFxConfig.MotorOutput
            // .withInverted(ANGLE_MOTOR_INVERTED)
            .withNeutralMode(NeutralModeValue.Brake);

        talonFxConfig.Feedback
            .withSensorToMechanismRatio(swerveConfig.anglePositionToRadiansConversionFactor)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
            .withFeedbackRotorOffset(getAbsoluteAngle());

        talonFxConfig.CurrentLimits
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(swerveConfig.angleMotorCurrentLimit);

        var slot0Configs = talonFxConfig.Slot0;

        slot0Configs.kP = 0.11;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.1;

        motorController.getConfigurator().apply(new TalonFXConfiguration());
        motorController.getConfigurator().apply(talonFxConfig, 0.05);
    }

    // Just about most of the time the motor encoder doesn't initialize properly, so we force it until it do
    public void init() {

        if (isInitialized) return;

        var currentAngle = getCurrentAngle();
        var absoluteAngle = getAbsoluteAngle();

        if (Math.abs(currentAngle - absoluteAngle) < 0.001) {
            isInitialized = true;
            return;
        }
    }

    public void setAngle(double desiredAngle) {

        double currentAngle = motorController.getPosition().getValueAsDouble();

        // Reset the NEO's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter anymore.
        if (motorController.getVelocity().getValueAsDouble() < swerveConfig.encoderResetMaxAngularVelocity) {
            if (++resetIteration >= swerveConfig.encoderResetIterations) {
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

    public boolean isInitialized() {
        return isInitialized;
    }
}