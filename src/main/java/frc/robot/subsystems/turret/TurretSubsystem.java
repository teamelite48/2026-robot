package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    boolean isAutoAimEnabled = true;
    boolean isAutoAimOn = false;

    // long lastSimulationPeriodicMillis = 0;
    // boolean isTurretEnabled = true;

    public TurretSubsystem() {

        var config = getMotorConfig();
        this.absoluteEncoderConfig = getAbsEncoderConfigTurret();

        motor = new Minion(config);
        // pidController = new PIDController(config.pidParameters.P, config.pidParameters.I, config.pidParameters.D);
        this.absoluteEncoder = new CanCoder(absoluteEncoderConfig);

        double absDegrees = getAbsoluteAngle();
        motor.setInitialPosition(absDegrees / 360.0);
        targetDegrees = clampTarget(absDegrees);

        // motor.setInitialPosition(getAbsoluteAngle() / 360.0);
        // targetDegrees = clampTarget(getPositionInDegrees());

        initDashboard();
    }

    @Override
    // public void periodic() {
    //     // 1. Update targetDegrees if AutoAim is active
    //     // This updates the 'targetDegrees' variable but doesn't move the motor yet
    //     if (RobotContainer.isAimAssistEnabled) {
    //         isManualControl = false;
    //         autoAim();
    //     }

    //     // 2. Safety: If a manual button is being held, skip the position control logic
    //     if (isManualControl) {
    //         return; 
    //     }

    //     // 3. Constant Position Holding
    //     // By calling this every 20ms, the turret will stay locked to 'targetDegrees'
    //     // even if the target is lost or the robot chassis spins.
    //     double targetRotations = targetDegrees / 360.0;
    //     motor.setMotionMagicPosition(targetRotations, 0.0);
    // }

    public void periodic() {
        if (RobotContainer.isAimAssistEnabled) {
            isManualControl = false; // Force manual mode OFF
            autoAim();
        }

        if (isManualControl) {
            // While a manual button is held, we do NOT call setMotionMagic.
            // The setMotor() calls from your manual methods handle the movement.
            return; 
        }

        // This is the "Active" state. 
        // If the motor isn't moving here, it means it's still thinking 
        // it's in the DutyCycle mode from the manual buttons.
        double targetRotations = targetDegrees / 360.0;
        motor.setMotionMagicPosition(targetRotations, 0.0);
    }

    // public void enableTurret() {
    //     isTurretEnabled = true;
    // }

    // public void disableTurret() {
    //     isTurretEnabled = false;
    // }

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
        if (isTargetAcquired()) {
            double error = RobotContainer.shooterVisionSubsystem.getXOffsetDegrees();
            double currentPos = getPositionInDegrees();
            
            // Use a temporary variable to see if the math is working
            double calculatedTarget = currentPos - error;
            
            // Update the global target
            targetDegrees = clampTarget(calculatedTarget);
            
            // If this prints the SAME number over and over while tx is 7, 
            // then clampTarget is the problem!
            // System.out.println("Target updating to: " + targetDegrees);
        }
    }

    public boolean isTargetAcquired(){
        return RobotContainer.shooterVisionSubsystem.hasTarget();
    }

    public void stop() {
        // 1. Tell the motor to stop spinning at a percentage
        motor.stop(); 
        
        // 2. IMPORTANT: Update the target to where we are RIGHT NOW
        // This prevents the turret from "snapping" back to an old target
        targetDegrees = getPositionInDegrees();
        
        // 3. Hand control back to the Motion Magic logic
        isManualControl = false;
    }

    public void moveToDegrees(Double degrees) {
        RobotContainer.disableAimAssist();
        isManualControl = false;
        targetDegrees = clampTarget(degrees);
    }

    public void moveToHome() {
        RobotContainer.disableAimAssist();
        isManualControl = false;
        moveToDegrees(HOME_POSITION);
    }

    public void rotateClockwise() {
        RobotContainer.disableAimAssist();
        isManualControl = true;
        setMotor(TurretConfig.clockwiseSpeed);
        // targetDegrees = clampTarget(targetDegrees + 2);
    }

    public void rotateCounterClockwise() {
        RobotContainer.disableAimAssist();
        isManualControl = true;
        setMotor(TurretConfig.counterClockwiseSpeed);
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

    private boolean isNearTarget() {
        return Math.abs(clampTarget(targetDegrees) - getPositionInDegrees()) < DEGREES_TOLERANCE;
    }

    private double getMotionMagicFeedForwardVolts() {
        double currentDeg = getPositionInDegrees();
        double targetDeg = clampTarget(targetDegrees);

        double sign = Math.signum(targetDeg - currentDeg);
        if (sign == 0.0) {
            return 0.0;
        }

        double ff = 0.0;

        if (currentDeg >= 10.0 && currentDeg <= 80.0) {
            ff = FEED_FORWARD_VOLTS;
        }
        else if (currentDeg >= 125.0 && currentDeg <= 145.0) {
            ff = FEED_FORWARD_VOLTS;
        }

        return ff * sign;
    }

    private boolean isStuckMovingToTarget() {
        double error = clampTarget(targetDegrees) - getPositionInDegrees();

        if (Math.abs(error) < DEGREES_TOLERANCE) {
            return false;
        }

        return Math.abs(motor.getVelocity()) < 0.01;
    }

    private boolean isTurretInitialized() {
        double turretDegrees = getPositionInDegrees();
        double absAngle = getAbsoluteAngle();

        // Use && to check if both conditions are true
        boolean turretInRange = (0.0 < turretDegrees) && (turretDegrees < TURRET_INIT_DEGREES);
        boolean absInRange = (0.0 < absAngle) && (absAngle < TURRET_INIT_DEGREES);

        return turretInRange && absInRange;
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
    }
}
