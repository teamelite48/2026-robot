package frc.robot.subsystems.drive;

import static frc.robot.components.swerve.lib.SwerveMath.TAU;

import frc.robot.components.encoders.absolute.lib.AbsoluteEncoderConfig;
import frc.robot.components.motors.lib.MotorConfig;
import frc.robot.components.swerve.lib.SwerveConfig;
import frc.robot.lib.PIDParameters;


public class DriveConfig {

    public static final int GYRO_ID = 5;
    public static final boolean IS_DRIVE_MOTOR_INVERTED = false;
    public static final boolean IS_ANGLE_MOTOR_INVERTED = true;
    public static final boolean IS_DRIVE_BRAKE_MODE_ENABLED = true;
    public static final boolean IS_ANGLE_BRAKE_MODE_ENABLED = false;

    // Set offset using Absolute Angle values from Elastic; always start with 0.0 & wheels straight when configuring.
    public static final double FRONT_LEFT_ANGLE_OFFSET_DEGREES = 68.0; //114.61; //294.83;
    public static final double FRONT_RIGHT_ANGLE_OFFSET_DEGREES = 202.1; //161.81; //343.21;
    public static final double REAR_LEFT_ANGLE_OFFSET_DEGREES = 332.7; //214.10; //34.57;
    public static final double REAR_RIGHT_ANGLE_OFFSET_DEGREES = 132.7; //229.39; //50.18;

    public static final PIDParameters ANGLE_CONTROLLER_PID_PARAMETERS = new PIDParameters(0.2, 0.0, 0);

    // Can remove if different solution in SwerveModule.java initDashboard()
    public static final int FRONT_LEFT_DRIVE_CAN_ID = 21;
    public static final int FRONT_RIGHT_DRIVE_CAN_ID = 23;
    public static final int REAR_LEFT_DRIVE_CAN_ID = 25;
    public static final int REAR_RIGHT_DRIVE_CAN_ID = 27;

    public static final int FRONT_LEFT_ANGLE_CAN_ID = 22;
    public static final int FRONT_RIGHT_ANGLE_CAN_ID = 24;
    public static final int REAR_LEFT_ANGLE_CAN_ID = 26;
    public static final int REAR_RIGHT_ANGLE_CAN_ID = 28;


    public static final SwerveConfig SWERVE_CONFIG = new SwerveConfig.Builder()
        .wheelDiameter(0.10033)
        .setTrackWidth(0.61595) //0.47625
        .wheelbase(0.47625) //0.61595
        .maxSpeed(5.193792)         // with_foc=16.08f/s=4.901184m/s; without_foc=17.4f/s=5.193792m/s
        .driveReduction(1.0 / 6.03)
        .angleReduction(1.0 / 26.0)
        .setAngleInversion(IS_ANGLE_MOTOR_INVERTED)
        .build();


    public static MotorConfig getAngleControllerConfigLeftFrontAngle() {
        var config = new MotorConfig(FRONT_LEFT_ANGLE_CAN_ID);
        config.isInverted = IS_ANGLE_MOTOR_INVERTED;
        config.isBrakeModeEnabled = IS_ANGLE_BRAKE_MODE_ENABLED;
        config.continuousWrap = false;
        config.positionConversionFactor = SWERVE_CONFIG.angleRadiansPerRotation();
        config.currentLimit = SWERVE_CONFIG.getAngleMotorCurrentLimit();
        // config.initialPosition = 0.0;
        config.pidParameters = ANGLE_CONTROLLER_PID_PARAMETERS;
        return config;
    }

    public static MotorConfig getAngleControllerConfigRightFrontAngle() {
        var config = new MotorConfig(FRONT_RIGHT_ANGLE_CAN_ID);
        config.isInverted = IS_ANGLE_MOTOR_INVERTED;
        config.isBrakeModeEnabled = IS_ANGLE_BRAKE_MODE_ENABLED;
        config.continuousWrap = false;
        config.positionConversionFactor = SWERVE_CONFIG.angleRadiansPerRotation();
        config.currentLimit = SWERVE_CONFIG.getAngleMotorCurrentLimit();
        // config.initialPosition = 0.0;
        config.pidParameters = ANGLE_CONTROLLER_PID_PARAMETERS;
        return config;
    }

    public static MotorConfig getAngleControllerConfigLeftRearAngle() {
        var config = new MotorConfig(REAR_LEFT_ANGLE_CAN_ID);
        config.isInverted = IS_ANGLE_MOTOR_INVERTED;
        config.isBrakeModeEnabled = IS_ANGLE_BRAKE_MODE_ENABLED;
        config.continuousWrap = false;
        config.positionConversionFactor = SWERVE_CONFIG.angleRadiansPerRotation();
        config.currentLimit = SWERVE_CONFIG.getAngleMotorCurrentLimit();
        // config.initialPosition = 0.0;
        config.pidParameters = ANGLE_CONTROLLER_PID_PARAMETERS;
        return config;
    }

    public static MotorConfig getAngleControllerConfigRightRearAngle() {
        var config = new MotorConfig(REAR_RIGHT_ANGLE_CAN_ID);
        config.isInverted = IS_ANGLE_MOTOR_INVERTED;
        config.isBrakeModeEnabled = IS_ANGLE_BRAKE_MODE_ENABLED;
        config.continuousWrap = false;
        config.positionConversionFactor = SWERVE_CONFIG.angleRadiansPerRotation();
        config.currentLimit = SWERVE_CONFIG.getAngleMotorCurrentLimit();
        // config.initialPosition = 0.0;
        config.pidParameters = ANGLE_CONTROLLER_PID_PARAMETERS;
        return config;
    }


    public static AbsoluteEncoderConfig getAbsEncoderConfigLeftFront() {
        var config = new AbsoluteEncoderConfig(31);
        config.positionConversionFactor = TAU;
        config.offset = Math.toRadians(FRONT_LEFT_ANGLE_OFFSET_DEGREES);
        config.isInverted = true;
        return config;
    }

    public static AbsoluteEncoderConfig getAbsEncoderConfigRightFront() {
        var config = new AbsoluteEncoderConfig(33);
        config.positionConversionFactor = TAU;
        config.offset = Math.toRadians(FRONT_RIGHT_ANGLE_OFFSET_DEGREES);
        config.isInverted = true;
        return config;
    }

    public static AbsoluteEncoderConfig getAbsEncoderConfigLeftRear() {
        var config = new AbsoluteEncoderConfig(35);
        config.positionConversionFactor = TAU;
        config.offset = Math.toRadians(REAR_LEFT_ANGLE_OFFSET_DEGREES);
        config.isInverted = true;
        return config;
    }

    public static AbsoluteEncoderConfig getAbsEncoderConfigRightRear() {
        var config = new AbsoluteEncoderConfig(37);
        config.positionConversionFactor = TAU;
        config.offset = Math.toRadians(REAR_RIGHT_ANGLE_OFFSET_DEGREES);
        config.isInverted = true;
        return config;
    }


    public static MotorConfig getDriveControllerConfigLeftFrontDrive() {
        var config = new MotorConfig(FRONT_LEFT_DRIVE_CAN_ID);
        config.isInverted = !IS_DRIVE_MOTOR_INVERTED;
        config.isBrakeModeEnabled = IS_DRIVE_BRAKE_MODE_ENABLED;
        config.positionConversionFactor = SWERVE_CONFIG.driveMetersPerRotation();
        config.currentLimit = SWERVE_CONFIG.getDriveMotorCurrentLimit();
        config.initialPosition = 0.0;
        config.enableFOC = true;
        return config;
    }

    public static MotorConfig getDriveControllerConfigRightFrontDrive() {
        var config = new MotorConfig(FRONT_RIGHT_DRIVE_CAN_ID);
        config.isInverted = IS_DRIVE_MOTOR_INVERTED;
        config.isBrakeModeEnabled = IS_DRIVE_BRAKE_MODE_ENABLED;
        config.positionConversionFactor = SWERVE_CONFIG.driveMetersPerRotation();
        config.currentLimit = SWERVE_CONFIG.getDriveMotorCurrentLimit();
        config.initialPosition = 0.0;
        config.enableFOC = true;
        return config;
    }

    public static MotorConfig getDriveControllerConfigLeftRearDrive() {
        var config = new MotorConfig(REAR_LEFT_DRIVE_CAN_ID);
        config.isInverted = IS_DRIVE_MOTOR_INVERTED;
        config.isBrakeModeEnabled = IS_DRIVE_BRAKE_MODE_ENABLED;
        config.positionConversionFactor = SWERVE_CONFIG.driveMetersPerRotation();
        config.currentLimit = SWERVE_CONFIG.getDriveMotorCurrentLimit();
        config.initialPosition = 0.0;
        config.enableFOC = true;
        return config;
    }

    public static MotorConfig getDriveControllerConfigRightRearDrive() {
        var config = new MotorConfig(REAR_RIGHT_DRIVE_CAN_ID);
        config.isInverted = !IS_DRIVE_MOTOR_INVERTED;
        config.isBrakeModeEnabled = IS_DRIVE_BRAKE_MODE_ENABLED;
        config.positionConversionFactor = SWERVE_CONFIG.driveMetersPerRotation();
        config.currentLimit = SWERVE_CONFIG.getDriveMotorCurrentLimit();
        config.initialPosition = 0.0;
        config.enableFOC = true;
        return config;
    }
}
