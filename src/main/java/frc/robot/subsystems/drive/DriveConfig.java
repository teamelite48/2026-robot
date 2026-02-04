package frc.robot.subsystems.drive;

import frc.robot.components.controllers.angle.lib.AngleControllerConfig;
import frc.robot.components.motors.lib.MotorConfig;
import frc.robot.components.swerve.lib.SwerveConfig;
import frc.robot.lib.PIDParameters;

public class DriveConfig {

    public static final int GYRO_ID = 5;
    public static final boolean IS_DRIVE_MOTOR_INVERTED = true;
    public static final boolean IS_DRIVE_BRAKE_MODE_ENABLED = true;

    // Set offset using Absolute Angle values from Elastic; always start with 0.0 & wheels straight when configuring.
    public static final double FRONT_LEFT_ANGLE_OFFSET_DEGREES = 65.30;
    public static final double FRONT_RIGHT_ANGLE_OFFSET_DEGREES = 17.49;
    public static final double REAR_LEFT_ANGLE_OFFSET_DEGREES = 325.46;
    public static final double REAR_RIGHT_ANGLE_OFFSET_DEGREES = 130.86;

    // Can remove if different solution in SwerveModule.java initDashboard()
    public static final int FRONT_LEFT_DRIVE_CAN_ID = 21;
    public static final int FRONT_RIGHT_DRIVE_CAN_ID = 23;
    public static final int REAR_LEFT_DRIVE_CAN_ID = 25;
    public static final int REAR_RIGHT_DRIVE_CAN_ID = 27;


    public static SwerveConfig getSwerveConfig() {
        var config = new SwerveConfig();
        config.wheelDiameterMeters = 0.10033;
        config.trackWidthMeters = 0.47625;
        config.wheelbaseMeters = 0.61595;
        config.maxMetersPerSecond = 5.193792;  // with_foc=16.08f/s=4.901184m/s; without_foc=17.4f/s=5.193792m/s
        config.isAngleMotorInverted = false;
        config.driveMotorReduction = (1.0 / 6.03);
        config.angleMotorReduction = (1.0 / 26.0);
        config.movingRotationPid = new PIDParameters(0.005, 0.0001, 0);
        config.standingRotationPid = new PIDParameters(0.01, 0.01, 0.0);
        return config;
    }


    public static AngleControllerConfig getAngleControllerConfigLeftFrontAngle() {
        var config = new AngleControllerConfig(22, 31);
        config.angleOffsetDegrees = FRONT_LEFT_ANGLE_OFFSET_DEGREES;
        return config;
    }

    public static AngleControllerConfig getAngleControllerConfigRightFrontAngle() {
        var config = new AngleControllerConfig(24, 33);
        config.angleOffsetDegrees = FRONT_RIGHT_ANGLE_OFFSET_DEGREES;
        return config;
    }

    public static AngleControllerConfig getAngleControllerConfigLeftRearAngle() {
        var config = new AngleControllerConfig(26, 35);
        config.angleOffsetDegrees = REAR_LEFT_ANGLE_OFFSET_DEGREES;
        return config;
    }

    public static AngleControllerConfig getAngleControllerConfigRightRearAngle() {
        var config = new AngleControllerConfig(28, 37);
        config.angleOffsetDegrees = REAR_RIGHT_ANGLE_OFFSET_DEGREES;
        return config;
    }


    public static MotorConfig getDriveControllerConfigLeftFrontDrive() {
        var config = new MotorConfig(FRONT_LEFT_DRIVE_CAN_ID);
        config.isInverted = IS_DRIVE_MOTOR_INVERTED;
        config.isBrakeModeEnabled = IS_DRIVE_BRAKE_MODE_ENABLED;
        config.positionConversionFactor = getSwerveConfig().drivePositionToMetersConversionFactor;
        config.currentLimit = getSwerveConfig().driveMotorCurrentLimit;
        config.initialPosition = 0.0;
        return config;
    }

    public static MotorConfig getDriveControllerConfigRightFrontDrive() {
        var config = new MotorConfig(FRONT_RIGHT_DRIVE_CAN_ID);
        config.isInverted = IS_DRIVE_MOTOR_INVERTED;
        config.isBrakeModeEnabled = IS_DRIVE_BRAKE_MODE_ENABLED;
        config.positionConversionFactor = getSwerveConfig().drivePositionToMetersConversionFactor;
        config.currentLimit = getSwerveConfig().driveMotorCurrentLimit;
        config.initialPosition = 0.0;
        return config;
    }

    public static MotorConfig getDriveControllerConfigLeftRearDrive() {
        var config = new MotorConfig(REAR_LEFT_DRIVE_CAN_ID);
        config.isInverted = IS_DRIVE_MOTOR_INVERTED;
        config.isBrakeModeEnabled = IS_DRIVE_BRAKE_MODE_ENABLED;
        config.positionConversionFactor = getSwerveConfig().drivePositionToMetersConversionFactor;
        config.currentLimit = getSwerveConfig().driveMotorCurrentLimit;
        config.initialPosition = 0.0;
        return config;
    }

    public static MotorConfig getDriveControllerConfigRightRearDrive() {
        var config = new MotorConfig(REAR_RIGHT_DRIVE_CAN_ID);
        config.isInverted = IS_DRIVE_MOTOR_INVERTED;
        config.isBrakeModeEnabled = IS_DRIVE_BRAKE_MODE_ENABLED;
        config.positionConversionFactor = getSwerveConfig().drivePositionToMetersConversionFactor;
        config.currentLimit = getSwerveConfig().driveMotorCurrentLimit;
        config.initialPosition = 0.0;
        return config;
    }
}
