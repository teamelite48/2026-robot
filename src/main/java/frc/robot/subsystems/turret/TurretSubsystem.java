package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotContainer;
import frc.robot.components.motors.Neo550;
import frc.robot.components.motors.lib.Motor;
import frc.robot.subsystems.vision.VisionSubsystem;

import static frc.robot.subsystems.turret.TurretConfig.*;

public class TurretSubsystem extends SubsystemBase {

    final Motor motor;
    final PIDController pidController;

    final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    final NetworkTableEntry tx = table.getEntry("tx");
    final NetworkTableEntry tv = table.getEntry("tv");
    final NetworkTableEntry ledMode = table.getEntry("ledMode");
    final NetworkTableEntry camMode = table.getEntry("camMode");

    boolean isAutoAimEnabled = true;
    boolean isAutoAimOn = false;

    long lastSimulationPeriodicMillis = 0;
    boolean isTurretEnabled = true;

    public TurretSubsystem() {
        var config = getMotorConfig();

        motor = new Neo550(config);
        pidController = new PIDController(config.pidParameters.P, config.pidParameters.I, config.pidParameters.D);

        // initDashboard();
        
        ledMode.setNumber(3);
        camMode.setNumber(1);
    }

    @Override
    public void periodic() {

        if (isAutoAimOn) {
        ledMode.setNumber(3);
        camMode.setNumber(0);
        autoAim();
        }
        else {
        ledMode.setNumber(1);
        camMode.setNumber(1);
        }
    }

    public void simulationPeriodic() {

        long millisSinceLastPeriodic = System.currentTimeMillis() - lastSimulationPeriodicMillis;
        double elapsedSeconds = (millisSinceLastPeriodic / 1000.0);
        double rotationsSinceLastPeriodic = motor.getPosition() * elapsedSeconds * TurretConfig.nominalMotorRotationsPerSecond;

        motor.setPosition(motor.getPosition() + rotationsSinceLastPeriodic);

        lastSimulationPeriodicMillis = System.currentTimeMillis();
    }

    public void enableTurret() {
        isTurretEnabled = true;
    }

    public void disableTurret() {
        isTurretEnabled = false;
        turnAutoAimOff();
    }

    public void enableAutoAim() {
        isAutoAimEnabled = true;
    }

    public void disableAutoAim() {
        isAutoAimEnabled = false;
        isAutoAimOn = false;
    }

    public void turnAutoAimOn() {
        if (isAutoAimEnabled == false) return;

        isAutoAimOn = true;
    }

    public void turnAutoAimOff() {
        isAutoAimOn = false;
    }

    public void autoAim() {

        if(isTargetAcquired() == false) return;

        double error = tx.getDouble(0.0);
    }

    public boolean isTargetAcquired(){
        return tv.getDouble(0) == 1 ? true: false;
    }

    public void stop() {
        motor.stop();
    }

    public void moveToDegrees(Double degrees) {
        double motorSpeed = pidController.calculate(getPositionInDegrees(), degrees);
    }

    public void rotateClockwise() {
        turnAutoAimOff();
        setMotor(TurretConfig.clockwiseSpeed);
    }

    public void rotateCounterClockwise() {
        turnAutoAimOff();
        setMotor(TurretConfig.counterClockwiseSpeed);
    }

    public double getPositionInDegrees() {
        return motor.getPosition() * TurretConfig.degreesPerMotorRotation + TurretConfig.degreesAtCenter;
    }

  public void setMotor(double speed) {
        if (isTurretEnabled == false) {
            stop();
            return;
        }

        motor.setSpeed(speed);
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

        tab.addDouble("Turret tx", () -> tx.getDouble(0.0))
            .withPosition(0, 1)
            .withSize(2, 1);
    }
}
