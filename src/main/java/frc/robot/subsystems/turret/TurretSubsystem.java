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

    boolean isAutoAimEnabled = true;
    boolean isAutoAimOn = false;

    // long lastSimulationPeriodicMillis = 0;
    boolean isTurretEnabled = true;

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
    public void periodic() {
        if (!isTurretEnabled) return;

        if (isAutoAimOn) {
            autoAim();
        }

        double targetRot = clampTarget(targetDegrees) / 360.0;
        double ffVolts = isNearTarget() ? 0.0 : getMotionMagicFeedForwardVolts();

        motor.setMotionMagicPosition(targetRot, ffVolts);
    }

    public void enableTurret() {
        isTurretEnabled = true;
    }

    public void disableTurret() {
        isTurretEnabled = false;
    }

    public void autoAim() {

        if (isTargetAcquired() == false || RobotContainer.isAimAssistEnabled == false) return;

        double error = RobotContainer.shooterVisionSubsystem.getXOffsetDegrees();
        targetDegrees = clampTarget(getPositionInDegrees() - error);
    }

    public boolean isTargetAcquired(){
        return RobotContainer.shooterVisionSubsystem.hasTarget();
    }

    public void stop() {
        motor.stop();
    }

    public void moveToDegrees(Double degrees) {
        targetDegrees = clampTarget(degrees);
    }

    public void moveToHome() {
        RobotContainer.disableAimAssist();
        moveToDegrees(HOME_POSITION);
    }

    public void rotateClockwise() {
        RobotContainer.disableAimAssist();
        setMotor(TurretConfig.clockwiseSpeed);
        // targetDegrees = clampTarget(targetDegrees + 2);
    }

    public void rotateCounterClockwise() {
        RobotContainer.disableAimAssist();
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
        if (isTurretEnabled == false) {
            stop();
            return;
        }

        motor.setSpeed(speed);
    }

    private double clampTarget(double degrees) {
        return Math.max(CCW_LIMIT, Math.min(CW_LIMIT, degrees));
    }

    private boolean isNearTarget() {
        return Math.abs(clampTarget(targetDegrees) - getPositionInDegrees()) < FEED_FORWARD_DEGREES_TOLERANCE;
    }

    private double getMotionMagicFeedForwardVolts() {
        double currentDeg = getPositionInDegrees();
        double targetDeg = clampTarget(targetDegrees);

        double sign = Math.signum(targetDeg - currentDeg);
        if (sign == 0.0) {
            return 0.0;
        }

        double ff = 0.0;

        if (currentDeg >= 55.0 && currentDeg <= 80.0) {
            ff = FEED_FORWARD_VOLTS;
        }
        else if (currentDeg >= 125.0 && currentDeg <= 145.0) {
            ff = FEED_FORWARD_VOLTS;
        }

        return ff * sign;
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

        tab.addBoolean("Turret Enabled", () -> isTurretEnabled)
            .withPosition(4, 2)
            .withSize(1, 1);

        tab.addDouble("Motor Rotations", () -> motor.getPosition());
    }
}
