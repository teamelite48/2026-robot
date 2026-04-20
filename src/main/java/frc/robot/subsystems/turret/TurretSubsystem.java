package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.RobotContainer;
import frc.robot.components.encoders.absolute.CanCoder;
import frc.robot.components.encoders.absolute.lib.AbsoluteEncoderConfig;
import frc.robot.components.motors.Minion;
import frc.robot.components.motors.lib.Motor;

import static frc.robot.subsystems.turret.TurretConfig.*;
import static frc.robot.subsystems.shooter.ShooterConfig.*;

public class TurretSubsystem extends SubsystemBase {

    private final Motor motor;
    private final CanCoder absoluteEncoder;
    private final AbsoluteEncoderConfig absoluteEncoderConfig;
    private double targetDegrees = HOME_POSITION;
    public boolean isManualControl = true;
    private double manualSpeedRequest = 0.0;
    public static double compensatedDistance = 0.0;
    public static Translation2d compensatedTarget = new Translation2d();

    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(0.5);

    boolean isAutoAimEnabled = true;
    boolean isAutoAimOn = false;
    boolean automatedMove = false;
    double lastActiveI = -1.0;

    // long lastSimulationPeriodicMillis = 0;
    // boolean isTurretEnabled = true;

    public TurretSubsystem() {

        var config = getMotorConfig();
        this.absoluteEncoderConfig = getAbsEncoderConfigTurret();

        motor = new Minion(config);
        // pidController = new PIDController(config.pidParameters.P, config.pidParameters.I, config.pidParameters.D);
        this.absoluteEncoder = new CanCoder(absoluteEncoderConfig);

        // 1. Get Absolute Position and normalize (358 -> -2)
        double rawAbs = absoluteEncoder.getPosition();

        if (rawAbs > 180) {
            rawAbs -= 360;
        }
        // 2. Account for 9:1 Gear Ratio
        double turretDegrees = rawAbs / TURRET_GEAR_RATIO;

        // 3. Seed Motor and Target
        motor.setInitialPosition(turretDegrees / 360.0);
        targetDegrees = clampTarget(turretDegrees);

        // double absDegrees = getAbsoluteAngle();
        // motor.setInitialPosition(absDegrees / 360.0);
        // targetDegrees = clampTarget(absDegrees);

        // motor.setInitialPosition(getAbsoluteAngle() / 360.0);
        // targetDegrees = clampTarget(getPositionInDegrees());

        initDashboard();
    }

    @Override
    public void periodic() {

        double currentPos = getPositionInDegrees();
        // boolean shouldBeInSlackArea = (currentPos >= CCW_SOFT_MOVEMENT_LIMIT && currentPos <= CW_SOFT_MOVEMENT_LIMIT);
        double activeI = 0.0;

        // Only update the motor if the state has actually CHANGED
        // if (shouldBeInSlackArea != isInSlackArea) {
        //     isInSlackArea = shouldBeInSlackArea;
            
        //     if (isInSlackArea) {
        //         // Precision values applied ONCE
        //         motor.setPID(12.0, 0.02, 0.0, 0.55, 0.12);
        //     } else {
        //         // Travel values applied ONCE
        //         motor.setPID(37.5, 0.03, 0.05, 0.75, 0.12);
        //     }
        // }

        if (Math.abs(currentPos - targetDegrees) < 7.5) {
            activeI = 0.1;
        }
        else {
            activeI = 0.0;
        }

        if (lastActiveI != activeI) {
            motor.setPID(37.5, activeI, 0.05, 0.75, 0.12);
            lastActiveI = activeI;
        }

        if (RobotContainer.isAimAssistEnabled) {
            isManualControl = false;
            automatedMove = false;
            autoAim();

            double targetRotations = targetDegrees / 360.0;
            motor.setMotionMagicPosition(targetRotations, 0.0);
        }

        else if (isManualControl) {

            double limitedSpeed = speedLimiter.calculate(manualSpeedRequest);

            if (manualSpeedRequest == 0) {
                limitedSpeed = 0;
                speedLimiter.reset(0);
            }

            motor.setSpeed(limitedSpeed);
            targetDegrees = getPositionInDegrees();
            return;
        }

        else if (automatedMove) {
            double targetRotations = targetDegrees / 360.0;
            motor.setMotionMagicPosition(targetRotations, 0.0);
        }

        else {
            motor.stop();
        }

    }
    // USE IF WE HAVE TO CHANGE BACK TO TURRET MOUNTED LIMELIGHT
    // public void autoAim() {

    //     automatedMove = false;
    //     isManualControl = false;

    //     if (isTargetAcquired()) {

    //         double error = RobotContainer.shooterVisionSubsystem.getXOffsetDegrees();
    //         double currentPos = getPositionInDegrees();

    //         // Use a temporary variable to see if the math is working
    //         double calculatedTarget = currentPos - error;

    //         // Deadband tolerance for turret
    //         if (Math.abs(error) <= DEGREES_TOLERANCE) {
    //             targetDegrees = getPositionInDegrees();
    //         }
    //         else {
    //             // Update the global target
    //             targetDegrees = clampTarget(calculatedTarget);
    //         }
    //     }
    // }

    public void autoAim() {
        automatedMove = false;
        isManualControl = false; // Try bumping this slightly for rotation

        Pose2d robotPose = RobotContainer.driveSubsystem.getPose();
        ChassisSpeeds robotRelativeSpeeds = RobotContainer.driveSubsystem.getChassisSpeeds();
        Translation2d turretOffset = new Translation2d(-0.082885, 0.1778);

        double tangentialVx = -robotRelativeSpeeds.omegaRadiansPerSecond * turretOffset.getY();
        double tangentialVy = robotRelativeSpeeds.omegaRadiansPerSecond * turretOffset.getX();

        ChassisSpeeds turretRelativeSpeeds = new ChassisSpeeds(
            robotRelativeSpeeds.vxMetersPerSecond + tangentialVx,
            robotRelativeSpeeds.vyMetersPerSecond + tangentialVy,
            robotRelativeSpeeds.omegaRadiansPerSecond
        );

        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            turretRelativeSpeeds, 
            robotPose.getRotation()
        );

        Translation2d futureRobotTranslation = robotPose.getTranslation().plus(
            new Translation2d(
                fieldSpeeds.vxMetersPerSecond * STRAFE_PREDICTION_TIME,
                fieldSpeeds.vyMetersPerSecond * STRAFE_PREDICTION_TIME
            )
        );

        Rotation2d futureRotation = robotPose.getRotation().plus(Rotation2d.fromRadians(robotRelativeSpeeds.omegaRadiansPerSecond * ROTATION_PREDICTION_TIME));

        Translation2d targetLocation = getDynamicTarget();

        double distance = futureRobotTranslation.getDistance(targetLocation);
        double flightTime = (distance / AVERAGE_FUEL_VELOCITY) + LATENCY_COMPENSATION;

        double driftX = fieldSpeeds.vxMetersPerSecond * flightTime;
        double driftY = fieldSpeeds.vyMetersPerSecond * flightTime;

        compensatedTarget = new Translation2d(
            targetLocation.getX() - driftX,
            targetLocation.getY() - driftY
        );

        // double effectiveDistance = robotPose.getTranslation().getDistance(compensatedTarget);

        double effectiveDistance = futureRobotTranslation.getDistance(compensatedTarget);

        // If we are moving backwards (vx is negative)
        if (fieldSpeeds.vxMetersPerSecond < BACKWARDS_MOVEMENT_THRESHOLD) {
            // Because the flight time is 2s, the penalty is huge. 
            // Add 15-20% extra distance to the shooter's "perceived" target.
            compensatedDistance = effectiveDistance * BACKWARDS_BIAS_MODIFIER;
        }
        else {
            compensatedDistance = effectiveDistance;
        }

        // 1. Get field-relative direction to Hub
        // Translation2d robotToTarget = compensatedTarget.minus(robotPose.getTranslation());
        // double fieldRelativeAngle = robotToTarget.getAngle().getDegrees();

        Translation2d robotToTarget = compensatedTarget.minus(futureRobotTranslation);
        double fieldRelativeAngle = robotToTarget.getAngle().getDegrees();

        // 2. Subtract robot heading to get angle relative to robot front
        // double robotRelativeAngle = fieldRelativeAngle - robotPose.getRotation().getDegrees();

        double robotRelativeAngle = fieldRelativeAngle - futureRotation.getDegrees();

        // 3. Apply your "Zero is Right" offset
        // Since Right is -90 in field terms, we add 90 to make it your 0.
        double turretTarget = robotRelativeAngle + 90.0;

        // 4. INVERT for your motor's direction (CCW is Negative)
        // We multiply by -1 because your hardware moves opposite to standard math.
        turretTarget = turretTarget * -1.0;

        // 5. Wrap to stay within your [-270, 270] hardware limits
        // while (turretTarget > CW_LIMIT) turretTarget -= 360;
        // while (turretTarget < CCW_LIMIT) turretTarget += 360;
        double currentAngle = getPositionInDegrees();
        double adjustedTarget = currentAngle + MathUtil.inputModulus(turretTarget - currentAngle, -180, 180);

        if (adjustedTarget > CW_LIMIT && (adjustedTarget - 360) > CCW_LIMIT) {
            adjustedTarget -= 360;
        }
        else if (adjustedTarget < CCW_LIMIT && (adjustedTarget + 360) < CW_LIMIT) {
            adjustedTarget += 360;
        }

        // 6. Apply final clamps
        // targetDegrees = clampTarget(turretTarget);
        targetDegrees = MathUtil.clamp(adjustedTarget, CCW_LIMIT, CW_LIMIT);

        SmartDashboard.putNumber("Debug/Flight Time", flightTime);
        SmartDashboard.putNumber("Debug/Drift X", driftX);
        SmartDashboard.putNumber("vx Meters/s", fieldSpeeds.vxMetersPerSecond);

    }

    public Translation2d getDynamicTarget() {
        Pose2d robotPose = RobotContainer.driveSubsystem.getPose();
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

        if (isBlue) {
            // BLUE STRATEGY: If we are past the trench line, aim the turret for passing
            if (robotPose.getX() > BLUE_TRENCH_LINE) {
                return (robotPose.getY() >= Y_CENTER_LINE) ? BLUE_LEFT_PASS_AREA : BLUE_RIGHT_PASS_AREA;
            }
            return BLUE_HUB_LOCATION; // Standard scoring
        } else {
            // RED STRATEGY: If we are past the trench line, aim the turret for passing
            if (robotPose.getX() < RED_TRENCH_LINE) {
                // Note: "Left" and "Right" are relative to the driver's view!
                return (robotPose.getY() >= Y_CENTER_LINE) ? RED_RIGHT_PASS_AREA : RED_LEFT_PASS_AREA;
            }
            return RED_HUB_LOCATION; // Standard scoring
        }
    }

    public boolean isTargetAcquired(){
        return Math.abs(getPositionInDegrees() - targetDegrees) < DEGREES_TOLERANCE;
    }

    public void stop() {
        // 1. Tell the motor to stop spinning at a percentage
        // motor.stop();

        // 2. IMPORTANT: Update the target to where we are RIGHT NOW
        // This prevents the turret from "snapping" back to an old target
        manualSpeedRequest = 0.0;
        targetDegrees = getPositionInDegrees();
        isManualControl = false;
    }

    public void moveToDegrees(Double degrees) {
        RobotContainer.disableAimAssist();
        isManualControl = false;
        automatedMove = true;
        targetDegrees = clampTarget(degrees);
    }

    public void moveToHome() {
        moveToDegrees(HOME_POSITION);
    }

    public void rotateClockwise() {
        RobotContainer.disableAimAssist();
        automatedMove = false;
        isManualControl = true;

        manualSpeedRequest = TurretConfig.clockwiseSpeed;

        // targetDegrees = clampTarget(targetDegrees + 2);
    }

    public void rotateCounterClockwise() {
        RobotContainer.disableAimAssist();
        automatedMove = false;
        isManualControl = true;

        manualSpeedRequest = TurretConfig.counterClockwiseSpeed;

        // targetDegrees = clampTarget(targetDegrees - 2);
    }

    public double getPositionInDegrees() {
        return motor.getPosition() * 360.0;
    }

    public double getAbsoluteAngle() {
        return absoluteEncoder.getPosition();
    }

    public double getTargetDegrees() {
        return targetDegrees;
    }

    public void setMotor(double speed) {
        // if (isTurretEnabled == false) {
        //     stop();
        //     return;
        // }

        motor.setSpeed(speed);
    }

    private double clampTarget(double degrees) {
        return Math.max(CCW_LIMIT, Math.min(CW_LIMIT, degrees));
    }

    private boolean isTurretInitialized() {
        double turretDegrees = getPositionInDegrees();
        double absAngle = getAbsoluteAngle();

        // Use && to check if both conditions are true
        boolean turretInRange = (0.0 < turretDegrees) && (turretDegrees < TURRET_INIT_DEGREES);
        boolean absInRange = (0.0 < absAngle) && (absAngle < TURRET_INIT_DEGREES);

        return turretInRange && absInRange;
    }

    public static Translation2d getTargetHub() {
        // This checks which alliance you are on so you aim at the correct Hub
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
            return RED_HUB_LOCATION;
        }
        return BLUE_HUB_LOCATION;
    }

    public void setManualOutput(double speed) {

        // Apply a deadband so the turret doesn't "drift" if the stick is old
        if (Math.abs(speed) < MANUAL_DEADBAND) {
            manualSpeedRequest = 0;
            return;
        }
        else {
            isManualControl = true;
            automatedMove = false;
            RobotContainer.disableAimAssist();

            // Optional: Square the input for smoother control
            // manualSpeedRequest = (speed * Math.abs(speed));
            manualSpeedRequest = speed;
        }
    }

    public void initDashboard() {
        var tab = Shuffleboard.getTab("Turret");

        // SmartDashboard.putNumber("Turret Degrees", getPositionInDegrees());
        // SmartDashboard.putNumber("Turret tx", tx.getDouble(0.0));
        // SmartDashboard.putBoolean("Auto Aim Enabled", isAutoAimEnabled);
        // SmartDashboard.putBoolean("Auto Aim On", isAutoAimOn);
        // SmartDashboard.putBoolean("Target Acquired", isTargetAcquired());

        tab.addDouble("Turret Degrees", () -> getPositionInDegrees())
            .withPosition(0, 0)
            .withSize(2, 1);

        tab.addDouble("Turret tx", () -> RobotContainer.leftLimelight.getXOffsetDegrees())
            .withPosition(0, 1)
            .withSize(2, 1);

        tab.addDouble("Absolute Degrees", () -> getAbsoluteAngle())
            .withPosition(0, 2)
            .withSize(2, 1);

        tab.addDouble("Target Degrees", () -> getTargetDegrees())
            .withPosition(2, 2)
            .withSize(2, 1);

        tab.addDouble("Motor Rotations", () -> motor.getPosition());
        tab.addDouble("Motor Velocity", () -> motor.getVelocity());
        tab.addBoolean("Is Manual Mode Enabled", () -> isManualControl);
        tab.addBoolean("Is Turret Initialized", () -> isTurretInitialized());

        tab.addDouble("Distance to Hub (m)", () -> {
            Pose2d robotPose = RobotContainer.driveSubsystem.getPose();
            return robotPose.getTranslation().getDistance(getTargetHub());
        }
        );
    }
}
