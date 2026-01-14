package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.commands.SetElevatorHeight;

public class ElevatorCommands {
  final static ElevatorSubsystem elevatorSubsystem = RobotContainer.elevatorSubsystem;

  public static Command extend() {
    return Commands.run(() -> elevatorSubsystem.extend(), elevatorSubsystem);
  }

  public static Command retract() {
    return Commands.run(() -> elevatorSubsystem.retract(), elevatorSubsystem);
  }

  public static Command setHeight(double inches) {
    return new SetElevatorHeight(inches);
  }

  public static Command stop() {
    return Commands.run(() -> elevatorSubsystem.stop(), elevatorSubsystem);
  }
}
