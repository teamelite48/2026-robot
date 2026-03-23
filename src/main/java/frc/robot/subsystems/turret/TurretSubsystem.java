package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.RobotContainer;
import frc.robot.components.encoders.absolute.CanCoder;
import frc.robot.components.encoders.absolute.lib.AbsoluteEncoderConfig;
import frc.robot.components.motors.Minion;
import frc.robot.components.motors.lib.Motor;

import static frc.robot.subsystems.turret.TurretConfig.*;

public class TurretSubsystem extends SubsystemBase {

    private final Motor motor;
    private final CanCoder absoluteEncoder;
    private final AbsoluteEncoderConfig absoluteEncoderConfig;
    private double targetDegrees = HOME_POSITION;
    public boolean isManualControl = true;
    private double manualSpeedRequest = 0.0;

    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(2.0);

    boolean isAutoAimEnabled = true;
    boolean isAutoAimOn = false;
    boolean automatedMove = false;

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
        if (RobotContainer.isAimAssistEnabled) {
            isManualControl = false;
            automatedMove = false;
            autoAim();

            double targetRotations = targetDegrees / 360.0;
            motor.setMotionMagicPosition(targetRotations, 0.0);
        } 
        
        else if (isManualControl) {

            double limitedSpeed = speedLimiter.calculate(manualSpeedRequest);
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

    // public void autoAim() {

    //     //if (isTargetAcquired() == false || RobotContainer.isAimAssistEnabled == false) return;

    //     if (isTargetAcquired() && RobotContainer.isAimAssistEnabled) {

    //     double error = RobotContainer.shooterVisionSubsystem.getXOffsetDegrees();
    //     targetDegrees = clampTarget(getPositionInDegrees() - error);
    //     }
    //     else {
    //         return;
    //     }
    // }

    public void autoAim() {

        automatedMove = false;
        isManualControl = false;

        if (isTargetAcquired()) {

            double error = RobotContainer.shooterVisionSubsystem.getXOffsetDegrees();
            double currentPos = getPositionInDegrees();
            
            // Use a temporary variable to see if the math is working
            double calculatedTarget = currentPos - error;
            
            // Deadband tolerance for turret
            if (Math.abs(error) <= DEGREES_TOLERANCE) {
                targetDegrees = getPositionInDegrees();
            }
            else {
                // Update the global target
                targetDegrees = clampTarget(calculatedTarget);
            }
        }
    }

    public boolean isTargetAcquired(){
        return RobotContainer.shooterVisionSubsystem.hasTarget();
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

        tab.addDouble("Turret tx", () -> RobotContainer.shooterVisionSubsystem.getXOffsetDegrees())
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
        });
    }
}
