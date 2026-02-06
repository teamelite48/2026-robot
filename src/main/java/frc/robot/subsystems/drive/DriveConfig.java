package frc.robot.subsystems.drive;

import static frc.robot.components.swerve.lib.SwerveMath.TAU;

import frc.robot.components.encoders.absolute.lib.AbsoluteEncoderConfig;
import frc.robot.components.motors.lib.MotorConfig;
import frc.robot.components.swerve.lib.SwerveConfig;
import frc.robot.lib.PIDParameters;

public class DriveConfig {

    public static final int GYRO_ID = 5;
    public static final boolean IS_MOTOR_INVERTED = true;
    public static final boolean IS_DRIVE_BRAKE_MODE_ENABLED = true;
    public static final boolean IS_ANGLE_BRAKE_MODE_ENABLED = false;

    // Set offset using Absolute Angle values from Elastic; always start with 0.0 & wheels straight when configuring.
    public static final double FRONT_LEFT_ANGLE_OFFSET_DEGREES = 294.83;
    public static final double FRONT_RIGHT_ANGLE_OFFSET_DEGREES = 343.21;
    public static final double REAR_LEFT_ANGLE_OFFSET_DEGREES = 34.57;
    public static final double REAR_RIGHT_ANGLE_OFFSET_DEGREES = 50.18;

    //Backup values
    //public static final double FRONT_LEFT_ANGLE_OFFSET_DEGREES = 65.30;
    //public static final double FRONT_RIGHT_ANGLE_OFFSET_DEGREES = 17.49;
    //public static final double REAR_LEFT_ANGLE_OFFSET_DEGREES = 325.46;
    //public static final double REAR_RIGHT_ANGLE_OFFSET_DEGREES = 130.86;

    // Can remove if different solution in SwerveModule.java initDashboard()
    public static final int FRONT_LEFT_DRIVE_CAN_ID = 21;
    public static final int FRONT_RIGHT_DRIVE_CAN_ID = 23;
    public static final int REAR_LEFT_DRIVE_CAN_ID = 25;
    public static final int REAR_RIGHT_DRIVE_CAN_ID = 27;

    public static final int FRONT_LEFT_ANGLE_CAN_ID = 22;
    public static final int FRONT_RIGHT_ANGLE_CAN_ID = 24;
    public static final int REAR_LEFT_ANGLE_CAN_ID = 26;
    public static final int REAR_RIGHT_ANGLE_CAN_ID = 28;


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


    public static MotorConfig getAngleControllerConfigLeftFrontAngle() {
        var config = new MotorConfig(FRONT_LEFT_ANGLE_CAN_ID);
        config.isInverted = !IS_MOTOR_INVERTED;   //note the inversion
        config.isBrakeModeEnabled = IS_ANGLE_BRAKE_MODE_ENABLED;
        config.positionConversionFactor = getSwerveConfig().anglePositionToRadiansConversionFactor;
        config.currentLimit = getSwerveConfig().driveMotorCurrentLimit;
        config.initialPosition = 0.0;
        return config;
    }

    public static MotorConfig getAngleControllerConfigRightFrontAngle() {
        var config = new MotorConfig(FRONT_RIGHT_ANGLE_CAN_ID);
        config.isInverted = !IS_MOTOR_INVERTED;   //note the inversion
        config.isBrakeModeEnabled = IS_ANGLE_BRAKE_MODE_ENABLED;
        config.positionConversionFactor = getSwerveConfig().anglePositionToRadiansConversionFactor;
        config.currentLimit = getSwerveConfig().driveMotorCurrentLimit;
        config.initialPosition = 0.0;
        return config;
    }

    public static MotorConfig getAngleControllerConfigLeftRearAngle() {
        var config = new MotorConfig(REAR_LEFT_ANGLE_CAN_ID);
        config.isInverted = !IS_MOTOR_INVERTED;   //note the inversion
        config.isBrakeModeEnabled = IS_ANGLE_BRAKE_MODE_ENABLED;
        config.positionConversionFactor = getSwerveConfig().anglePositionToRadiansConversionFactor;
        config.currentLimit = getSwerveConfig().driveMotorCurrentLimit;
        config.initialPosition = 0.0;
        return config;
    }

    public static MotorConfig getAngleControllerConfigRightRearAngle() {
        var config = new MotorConfig(REAR_RIGHT_ANGLE_CAN_ID);
        config.isInverted = !IS_MOTOR_INVERTED;   //note the inversion
        config.isBrakeModeEnabled = IS_ANGLE_BRAKE_MODE_ENABLED;
        config.positionConversionFactor = getSwerveConfig().anglePositionToRadiansConversionFactor;
        config.currentLimit = getSwerveConfig().driveMotorCurrentLimit;
        config.initialPosition = 0.0;
        return config;
    }


    public static AbsoluteEncoderConfig getAbsEncoderConfigLeftFront() {
        var config = new AbsoluteEncoderConfig(31);
        config.positionConversionFactor = TAU;
        config.offset = FRONT_LEFT_ANGLE_OFFSET_DEGREES;
        return config;
    }

    public static AbsoluteEncoderConfig getAbsEncoderConfigRightFront() {
        var config = new AbsoluteEncoderConfig(33);
        config.positionConversionFactor = TAU;
        config.offset = FRONT_RIGHT_ANGLE_OFFSET_DEGREES;
        return config;
    }

    public static AbsoluteEncoderConfig getAbsEncoderConfigLeftRear() {
        var config = new AbsoluteEncoderConfig(35);
        config.positionConversionFactor = TAU;
        config.offset = REAR_LEFT_ANGLE_OFFSET_DEGREES;
        return config;
    }

    public static AbsoluteEncoderConfig getAbsEncoderConfigRightRear() {
        var config = new AbsoluteEncoderConfig(37);
        config.positionConversionFactor = TAU;
        config.offset = REAR_RIGHT_ANGLE_OFFSET_DEGREES;
        return config;
    }


    public static MotorConfig getDriveControllerConfigLeftFrontDrive() {
        var config = new MotorConfig(FRONT_LEFT_DRIVE_CAN_ID);
        config.isInverted = IS_MOTOR_INVERTED;
        config.isBrakeModeEnabled = IS_DRIVE_BRAKE_MODE_ENABLED;
        config.positionConversionFactor = getSwerveConfig().drivePositionToMetersConversionFactor;
        config.currentLimit = getSwerveConfig().driveMotorCurrentLimit;
        config.initialPosition = 0.0;
        return config;
    }

    public static MotorConfig getDriveControllerConfigRightFrontDrive() {
        var config = new MotorConfig(FRONT_RIGHT_DRIVE_CAN_ID);
        config.isInverted = IS_MOTOR_INVERTED;
        config.isBrakeModeEnabled = IS_DRIVE_BRAKE_MODE_ENABLED;
        config.positionConversionFactor = getSwerveConfig().drivePositionToMetersConversionFactor;
        config.currentLimit = getSwerveConfig().driveMotorCurrentLimit;
        config.initialPosition = 0.0;
        return config;
    }

    public static MotorConfig getDriveControllerConfigLeftRearDrive() {
        var config = new MotorConfig(REAR_LEFT_DRIVE_CAN_ID);
        config.isInverted = IS_MOTOR_INVERTED;
        config.isBrakeModeEnabled = IS_DRIVE_BRAKE_MODE_ENABLED;
        config.positionConversionFactor = getSwerveConfig().drivePositionToMetersConversionFactor;
        config.currentLimit = getSwerveConfig().driveMotorCurrentLimit;
        config.initialPosition = 0.0;
        return config;
    }

    public static MotorConfig getDriveControllerConfigRightRearDrive() {
        var config = new MotorConfig(REAR_RIGHT_DRIVE_CAN_ID);
        config.isInverted = IS_MOTOR_INVERTED;
        config.isBrakeModeEnabled = IS_DRIVE_BRAKE_MODE_ENABLED;
        config.positionConversionFactor = getSwerveConfig().drivePositionToMetersConversionFactor;
        config.currentLimit = getSwerveConfig().driveMotorCurrentLimit;
        config.initialPosition = 0.0;
        return config;
    }
}
