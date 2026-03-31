// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

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
import frc.robot.components.controllers.angle.TalonFxAngleController;
import frc.robot.components.controllers.drive.TalonFxDriveController;
import frc.robot.components.swerve.SwerveModule;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

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

    // private final SwerveConfig swerveConfig;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter rotationLimiter;

    // private final PIDController movingRoationPidController;
    // private final PIDController standingRoationPidController;

    private Gear gear = Gear.High;

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    final TalonFxAngleController frontLeftAngle;
    final TalonFxAngleController frontRightAngle;
    final TalonFxAngleController backLeftAngle;
    final TalonFxAngleController backRightAngle;

    final TalonFxDriveController frontLeftDrive;
    final TalonFxDriveController frontRightDrive;
    final TalonFxDriveController backLeftDrive;
    final TalonFxDriveController backRightDrive;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final Field2d field = new Field2d();

    private double currX, currY, currRotation = 0.0;

    public DriveSubsystem() {

        // this.swerveConfig = getSwerveConfig();

        xLimiter = new SlewRateLimiter(SWERVE_CONFIG.slewRate);
        yLimiter = new SlewRateLimiter(SWERVE_CONFIG.slewRate);
        rotationLimiter = new SlewRateLimiter(SWERVE_CONFIG.slewRate);
        // movingRoationPidController = new PIDController(swerveConfig.movingRotationPid.P, swerveConfig.movingRotationPid.I, swerveConfig.movingRotationPid.D);
        // standingRoationPidController = new PIDController(swerveConfig.standingRotationPid.P, swerveConfig.standingRotationPid.I, swerveConfig.standingRotationPid.D);

        frontLeftAngle = new TalonFxAngleController(getAngleControllerConfigLeftFrontAngle(), getAbsEncoderConfigLeftFront());
        frontRightAngle = new TalonFxAngleController(getAngleControllerConfigRightFrontAngle(), getAbsEncoderConfigRightFront());
        backLeftAngle = new TalonFxAngleController(getAngleControllerConfigLeftRearAngle(), getAbsEncoderConfigLeftRear());
        backRightAngle = new TalonFxAngleController(getAngleControllerConfigRightRearAngle(), getAbsEncoderConfigRightRear());

        frontLeftDrive = new TalonFxDriveController(SWERVE_CONFIG, getDriveControllerConfigLeftFrontDrive());
        frontRightDrive = new TalonFxDriveController(SWERVE_CONFIG, getDriveControllerConfigRightFrontDrive());
        backLeftDrive = new TalonFxDriveController(SWERVE_CONFIG, getDriveControllerConfigLeftRearDrive());
        backRightDrive = new TalonFxDriveController(SWERVE_CONFIG, getDriveControllerConfigRightRearDrive());

        frontLeft = new SwerveModule(SWERVE_CONFIG, frontLeftDrive, frontLeftAngle);
        frontRight = new SwerveModule(SWERVE_CONFIG, frontRightDrive, frontRightAngle);
        backLeft = new SwerveModule(SWERVE_CONFIG, backLeftDrive, backLeftAngle);
        backRight = new SwerveModule(SWERVE_CONFIG, backRightDrive, backRightAngle);

        frontLeft.init();
        frontRight.init();
        backLeft.init();
        backRight.init();

        kinematics = new SwerveDriveKinematics(
            new Translation2d(SWERVE_CONFIG.getTrackWidthMeters() / 2.0, SWERVE_CONFIG.getWheelBaseMeters() / 2.0),
            new Translation2d(SWERVE_CONFIG.getTrackWidthMeters() / 2.0, -SWERVE_CONFIG.getWheelBaseMeters() / 2.0),
            new Translation2d(-SWERVE_CONFIG.getTrackWidthMeters() / 2.0, SWERVE_CONFIG.getWheelBaseMeters() / 2.0),
            new Translation2d(-SWERVE_CONFIG.getTrackWidthMeters() / 2.0, -SWERVE_CONFIG.getWheelBaseMeters() / 2.0)
        );

        odometry = new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        });

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
                new PIDConstants(0.8, 0.0, 0.0),
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
        field.setRobotPose(getPose());
    }

    public void drive(double x, double y, double rotation) {

        double speedModifier = SWERVE_CONFIG.getMaxGearSpeed();

        if (gear == Gear.Low) {
            speedModifier = SWERVE_CONFIG.getLowGearSpeed();
        }

        var vx = -yLimiter.calculate(y * speedModifier) * SWERVE_CONFIG.getMaxMetersPerSecond();
        var vy = -xLimiter.calculate(x * speedModifier) * SWERVE_CONFIG.getMaxMetersPerSecond();
        // System.out.println("vx="+ vx + "\tvy="+vy);

        var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vx,
            vy,
            -rotationLimiter.calculate(rotation * speedModifier) * SWERVE_CONFIG.getMaxAngularMetersPerSecond(),
            Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble())
        );

        setSwerveModuleStates(chassisSpeeds);
    }

    public void driveRobotRelative(double x, double y) {

        var chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            yLimiter.calculate(y) * SWERVE_CONFIG.getMaxMetersPerSecond(),
            xLimiter.calculate(x) * SWERVE_CONFIG.getMaxMetersPerSecond(),
            rotationLimiter.calculate(0) * SWERVE_CONFIG.getMaxAngularMetersPerSecond(),
            Rotation2d.fromDegrees(0.0)
        );

        setSwerveModuleStates(chassisSpeeds);
    }

    public void driveWithRotationAssist(double x, double y, double rotationError) {

        currX = x;
        currY = y;
        currRotation = rotationError;

        double speedModifier = SWERVE_CONFIG.maxGearSpeed;

        if (gear == Gear.Low) {
            speedModifier = SWERVE_CONFIG.lowGearSpeed;
        }

        var rotation = 0.0;

        var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            yLimiter.calculate(y * speedModifier) * SWERVE_CONFIG.getMaxMetersPerSecond(),
            xLimiter.calculate(x * speedModifier) * SWERVE_CONFIG.getMaxMetersPerSecond(),
            rotation * SWERVE_CONFIG.getMaxAngularMetersPerSecond(),
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

    public double getStrafeSpeed() {
        return SWERVE_CONFIG.strafeSpeed;
    }

    private SwerveModuleState[] calculateSwerveModuleStates(ChassisSpeeds chassisSpeeds) {

        var swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SWERVE_CONFIG.getMaxMetersPerSecond());

        return swerveModuleStates;
    }

    private void setSwerveModuleStates(ChassisSpeeds chassisSpeeds) {

        var states = calculateSwerveModuleStates(chassisSpeeds);

        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }


    public ChassisSpeeds getChassisSpeeds() {
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

    public Pose2d getPose() {
        return odometry.getPoseMeters();
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

        driveTab.addDouble("Robot X (m)", () -> odometry.getPoseMeters().getX())
            .withPosition(0, 3);

        driveTab.addDouble("Robot Y (m)", () -> odometry.getPoseMeters().getY())
            .withPosition(1, 3);

        driveTab.addDouble("Robot Heading (deg)", () -> odometry.getPoseMeters().getRotation().getDegrees())
            .withPosition(2, 3);

        // // This pulls the X coordinate from the getPose() we just made
        // driveTab.addDouble("Robot X (m)", () -> getPose().getX());

        // // This pulls the Y coordinate
        // driveTab.addDouble("Robot Y (m)", () -> getPose().getY());

        // This shows the distance to the Hub in meters
        driveTab.addDouble("Dist to Hub (m)", () ->
            getPose().getTranslation().getDistance(RobotContainer.turretSubsystem.getTargetHub()));

        driveTab.add("Field", field)
            .withPosition(0, 4)
            .withSize(6, 4);
    }
}