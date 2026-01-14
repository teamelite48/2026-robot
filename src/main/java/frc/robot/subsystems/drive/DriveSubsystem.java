// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.components.SwerveModule;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import static frc.robot.subsystems.drive.DriveConfig.*;

public class DriveSubsystem extends SubsystemBase{

    public enum Gear {
        Low,
        High
    }

    private final Pigeon2 gyro = new Pigeon2(GYRO_ID);

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(SLEW_RATE);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(SLEW_RATE);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SLEW_RATE);

    private Gear gear = Gear.High;

    private final SwerveModule frontLeft = new SwerveModule(
        FRONT_LEFT_DRIVE_MOTOR_ID,
        FRONT_LEFT_ANGLE_MOTOR_ID,
        FRONT_LEFT_ANGLE_ENCODER_ID,
        FRONT_LEFT_ANGLE_OFFSET_DEGREES
    );

    private final SwerveModule frontRight = new SwerveModule(
        FRONT_RIGHT_DRIVE_MOTOR_ID,
        FRONT_RIGHT_ANGLE_MOTOR_ID,
        FRONT_RIGHT_ANGLE_ENCODER_ID,
        FRONT_RIGHT_ANGLE_OFFSET_DEGREES
    );

    private final SwerveModule backLeft = new SwerveModule(
        BACK_LEFT_DRIVE_MOTOR_ID,
        BACK_LEFT_ANGLE_MOTOR_ID,
        BACK_LEFT_ANGLE_ENCODER_ID,
        BACK_LEFT_ANGLE_OFFSET_DEGREES
    );

    private final SwerveModule backRight = new SwerveModule(
        BACK_RIGHT_DRIVE_MOTOR_ID,
        BACK_RIGHT_ANGLE_MOTOR_ID,
        BACK_RIGHT_ANGLE_ENCODER_ID,
        BACK_RIGHT_ANGLE_OFFSET_DEGREES
    );

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
        new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
        new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
        new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0)
    );

    private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
        kinematics,
        Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });

    private final PIDController movingRoationPidController = new PIDController(MOVING_ROTATION_PID.P, MOVING_ROTATION_PID.I, MOVING_ROTATION_PID.D);
    private final PIDController standingRoationPidController = new PIDController(STANDING_ROTATION_PID.P, STANDING_ROTATION_PID.I, STANDING_ROTATION_PID.D);

    private double currX, currY, currRotation = 0.0;

    public DriveSubsystem() {
        zeroGyro();
        configAutobuilder();
        setHighGear();
        initDashboard();
    }

    private void configAutobuilder() {

        RobotConfig config;

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        AutoBuilder.configure(
            () -> odometry.getPoseMeters(),
            this::resetOdometry,
            this::getChassisSpeeds,
            (speeds, feedforwards) -> setSwerveModuleStates(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(0.33, 0.0, 0.0),
                new PIDConstants(1.1, 0.0, 0.0)
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();

                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }

                return false;
            },

            this
        );
    }

    public void periodic() {
        updateOdometry();
        frontLeft.init();
        frontRight.init();
        backLeft.init();
        backRight.init();
    }

    public void drive(double x, double y, double rotation) {

        double speedModifier = MAX_OUTPUT;

        if (gear == Gear.Low) {
            speedModifier = LOW_GEAR_SPEED;
        }

        var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            -yLimiter.calculate(y * speedModifier) * MAX_METERS_PER_SECOND,
            -xLimiter.calculate(x * speedModifier) * MAX_METERS_PER_SECOND,
            -rotationLimiter.calculate(rotation * speedModifier) * MAX_ANGULAR_METERS_PER_SECOND,
            Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble())
        );

        setSwerveModuleStates(chassisSpeeds);
    }

    public void driveRobotRelative(double x, double y) {

        var chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            yLimiter.calculate(y) * MAX_METERS_PER_SECOND,
            xLimiter.calculate(x) * MAX_METERS_PER_SECOND,
            rotationLimiter.calculate(0) * MAX_ANGULAR_METERS_PER_SECOND,
            Rotation2d.fromDegrees(0.0)
        );

        setSwerveModuleStates(chassisSpeeds);
    }

    public void driveWithRotationAssist(double x, double y, double rotationError) {

        currX = x;
        currY = y;
        currRotation = rotationError;

        double speedModifier = MAX_OUTPUT;

        if (gear == Gear.Low) {
            speedModifier = LOW_GEAR_SPEED;
        }

        var rotation = 0.0;

        var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            yLimiter.calculate(y * speedModifier) * MAX_METERS_PER_SECOND,
            xLimiter.calculate(x * speedModifier) * MAX_METERS_PER_SECOND,
            rotation * MAX_ANGULAR_METERS_PER_SECOND,
            Rotation2d.fromDegrees(gyro.getYaw ().getValueAsDouble())
        );

        setSwerveModuleStates(chassisSpeeds);
    }

    public void setLowGear(){
        gear = Gear.Low;
    }

    public void setHighGear(){
        gear = Gear.High;
    }

    public void zeroGyro() {
        gyro.setYaw(0.0);
    }

    public void invertGyro() {
        gyro.setYaw(gyro.getYaw().getValueAsDouble() + 180.0);
    }

    public double getPitch() {
        return gyro.getRoll().getValueAsDouble();
    }

    public Gear getGear() {
        return gear;
    }

    private SwerveModuleState[] calculateSwerveModuleStates(ChassisSpeeds chassisSpeeds) {

        var swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_METERS_PER_SECOND);

        return swerveModuleStates;
    }

    private void setSwerveModuleStates(ChassisSpeeds chassisSpeeds) {

        var states = calculateSwerveModuleStates(chassisSpeeds);

        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }


    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        );
    }

    private void resetOdometry(Pose2d pose) {

        var degrees = pose.getRotation().getDegrees();

        gyro.setYaw(degrees);

        // var start = System.currentTimeMillis();

        // while (System.currentTimeMillis() - start < 25) {
        //     // wait ~1 cycle for the gyro to initialize
        // }

        odometry.resetPosition(
            Rotation2d.fromDegrees(degrees),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            },
            pose
        );
    }

    private void updateOdometry() {
        odometry.update(
            Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );
    }

    private void initDashboard() {

        var driveTab = Shuffleboard.getTab("Drive");

        driveTab.addDouble("Pitch", () -> getPitch())
            .withPosition(0, 0);

            driveTab.addDouble("Yaw", () -> gyro.getYaw().getValueAsDouble())
            .withPosition(1, 0);

        driveTab.addString("Odometry", () -> odometry.getPoseMeters().getTranslation().toString())
            .withPosition(2, 0)
            .withSize(2, 1);

        driveTab.addString("Chassis Speeds", () -> getChassisSpeeds().toString())
            .withPosition(4,0)
            .withSize(2, 1);

        driveTab.addString("Gear", () -> gear.toString());

        driveTab.addDouble("X", () -> currX);
        driveTab.addDouble("Y", () -> currY);
        driveTab.addDouble("Rotation", () -> currRotation);
    }
}