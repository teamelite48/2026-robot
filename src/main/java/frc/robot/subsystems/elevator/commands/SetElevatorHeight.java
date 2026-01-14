package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class SetElevatorHeight extends Command {

  ElevatorSubsystem elevatorSubsystem = RobotContainer.elevatorSubsystem;
  double targetHeight;

  public SetElevatorHeight(double inches) {
    this.targetHeight = inches;
    addRequirements(elevatorSubsystem);
  }

  public void initialize() {
    elevatorSubsystem.setHeight(targetHeight);
  }

  public void execute() {}

  public boolean isFinished() {
    return elevatorSubsystem.isAtTargetHeight();
  }
}
