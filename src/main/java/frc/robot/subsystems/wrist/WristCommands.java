package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.wrist.commands.FlipWristCommand;
import frc.robot.subsystems.wrist.commands.SendWristHomeCommand;
import frc.robot.subsystems.wrist.commands.SetWristPositionCommand;

public class WristCommands {

  final static WristSubsystem wristSubsystem = RobotContainer.wristSubsystem;
  private static WristPosition lastWristPosition;

  public static Command tiltUp() {
    return Commands.run(() -> wristSubsystem.tiltUp(), wristSubsystem);
  }

  public static Command tiltDown() {
    return Commands.run(() -> wristSubsystem.tiltDown(), wristSubsystem);
  }

  public static Command rotateClockwise() {
    return Commands.run(() -> wristSubsystem.rotateClockwise(), wristSubsystem);
  }

  public static Command rotateCounterClockwise() {
    return Commands.run(() -> wristSubsystem.rotateCounterClockwise(), wristSubsystem);
  }

  public static Command goHome() {
    return new SendWristHomeCommand();
  }

  public static Command flip() {
    return Commands.either(
      Commands.sequence(
        Commands.runOnce(() -> lastWristPosition = new WristPosition(wristSubsystem.getCurrentRotationDegrees(), wristSubsystem.getCurrentTiltDegrees())),
        goHome(),
        new FlipWristCommand(() -> lastWristPosition)
      ),
      Commands.none(),
      () -> RobotContainer.isWristFlippable
    );
  }

  public static Command setPosition(WristPosition wristPosition) {
    return new SetWristPositionCommand(wristPosition);
  }
}
