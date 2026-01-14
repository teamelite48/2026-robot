package frc.robot.subsystems.drive.components;


import com.ctre.phoenix6.configs.CANcoderConfiguration;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;

import static frc.robot.subsystems.drive.DriveConfig.*;
import static frc.robot.subsystems.drive.lib.SwerveMath.*;

public class SparkMaxAngleController {

    private final SparkMax motorController;
    private final SparkClosedLoopController pidController;
    private final RelativeEncoder motorEncoder;
    private final CANcoder absoluteEncoder;

    private double targetAngle = 0.0;
    private boolean isInitialized = false;

    public SparkMaxAngleController(int motorId, int absoluteEncoderId, double offsetDegrees) {

        absoluteEncoder = new CANcoder(absoluteEncoderId);

        var config = new CANcoderConfiguration();

        config.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0); // TODO: need to update logic because we no longer have 0 to 360 output. No clue what this means.
        config.MagnetSensor.withMagnetOffset(-(offsetDegrees / 360));
        config.MagnetSensor.withSensorDirection(ANGLE_MOTOR_ABSOLUTE_ENCODER_INVERSION);

        absoluteEncoder.getConfigurator().apply(config);


        motorController = new SparkMax(motorId, MotorType.kBrushless);
        motorEncoder = motorController.getEncoder();
        pidController = motorController.getClosedLoopController();

        var sparkMaxConfig = new SparkMaxConfig();

        sparkMaxConfig.voltageCompensation(NOMINAL_VOLTAGE);
        sparkMaxConfig.smartCurrentLimit(ANGLE_MOTOR_CURRENT_LIMIT);

        sparkMaxConfig.inverted(ANGLE_MOTOR_INVERTED);

        // TODO: 2024 => 2025
        // motorController.setPeriodicFramePeriod(SparkMax.PeriodicFrame.kStatus0, 100);
        // motorController.setPeriodicFramePeriod(SparkMax.PeriodicFrame.kStatus1, 20);
        // motorController.setPeriodicFramePeriod(SparkMax.PeriodicFrame.kStatus2, 20);

        sparkMaxConfig.idleMode(IdleMode.kCoast);

        sparkMaxConfig.encoder.positionConversionFactor(ANGLE_POSITION_TO_RADIANS_CONVERSION_FACTOR);
        sparkMaxConfig.encoder.velocityConversionFactor(ANGLE_POSITION_TO_RADIANS_CONVERSION_FACTOR / 60.0);
        motorEncoder.setPosition(getAbsoluteAngle());


        sparkMaxConfig.closedLoop.pid(1.0, 0.0, 0.0);
        sparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        motorController.configure(sparkMaxConfig, null, null); // TODO: null, null, wat?
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

        motorEncoder.setPosition(absoluteAngle);
    }

    public void setAngle(double desiredAngle) {

        if (isInitialized == false) return;

        double currentAngle = motorEncoder.getPosition();
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

        pidController.setReference(this.targetAngle, SparkMax.ControlType.kPosition);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getCurrentAngle() {
        return normalizeAngle(motorEncoder.getPosition());
    }

    public double getAbsoluteAngle() {
        return normalizeAngle(absoluteEncoder.getAbsolutePosition().getValueAsDouble() * TAU);
    }

    public boolean isInitialized() {
        return isInitialized;
    }
}