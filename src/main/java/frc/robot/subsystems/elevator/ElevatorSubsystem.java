package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.motors.Kraken;
import frc.robot.components.motors.lib.Motor;
import frc.robot.util.EliteMath;

import static frc.robot.subsystems.elevator.ElevatorConfig.*;

public class ElevatorSubsystem extends SubsystemBase {

  final Motor motor;
  final PIDController pidController;


  double targetHeight;
  double currentHeight;

  public ElevatorSubsystem() {

    var motorConfig = getMotorConfig();

    motor = new Kraken(motorConfig);
    pidController = new PIDController(ElevatorConfig.PID_PARAMETERS.P, ElevatorConfig.PID_PARAMETERS.I, ElevatorConfig.PID_PARAMETERS.D);

    targetHeight = motorConfig.initialPosition;
    currentHeight = motorConfig.initialPosition;

    initDashboard();
  }

  public void periodic() {
    currentHeight = motor.getPosition();

    var speed = EliteMath.clamp(
      pidController.calculate(currentHeight, targetHeight),
      ElevatorConfig.RETRACT_SPEED,
      ElevatorConfig.EXTEND_SPEED
    );

    motor.setSpeed(speed);
  }

  public void extend() {
    targetHeight = Math.min(targetHeight + ElevatorConfig.MANUAL_TARGET_MODIFIER, ElevatorConfig.MAX_HEIGHT);
  }

  public void retract() {
    targetHeight = Math.max(targetHeight - ElevatorConfig.MANUAL_TARGET_MODIFIER, ElevatorConfig.HOME_POSITIION);
  }

  public void setHeight(double inches) {
    targetHeight = inches;
  }

  public boolean isAtTargetHeight() {
    return Math.abs(targetHeight - currentHeight) <= ElevatorConfig.TARGET_THRESHOLD;
  }

  public void stop() {
    motor.stop();
  }

  public double getHeightInches() {
    return motor.getPosition();
  }

  private void initDashboard() {
    var elevatorTab = Shuffleboard.getTab("Elevator");

    elevatorTab.addDouble("Height", () -> motor.getPosition())
      .withPosition(0, 0)
      .withSize(2, 1);

    elevatorTab.addDouble("Target Height", () -> targetHeight)
      .withPosition(2, 0)
      .withSize(2, 1);
  }
}
