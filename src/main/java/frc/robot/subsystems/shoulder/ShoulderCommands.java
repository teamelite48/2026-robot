package frc.robot.subsystems.shoulder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shoulder.commands.SetShoulderAngleCommand;

public class ShoulderCommands {

  final static ShoulderSubsystem shoulderSubsystem = RobotContainer.shoulderSubsystem;

  public static Command tiltUp() {
    return Commands.run(() -> shoulderSubsystem.tiltUp(), shoulderSubsystem);
  }

  public static Command tiltDown() {
    return Commands.run(() -> shoulderSubsystem.tiltDown(), shoulderSubsystem);
  }

  public static Command stop() {
    return Commands.run(() -> shoulderSubsystem.stop(), shoulderSubsystem);
  }

  public static Command setAngle(double degrees) {
    return new SetShoulderAngleCommand(degrees);
  }
}
