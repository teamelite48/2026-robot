package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConfig.GamePiece;
import frc.robot.RobotContainer;

public class ClawCommands {

  final static ClawSubsystem clawSubsystem = RobotContainer.clawSubsystem;

  public static Command intake() {
    return Commands.either(
      Commands.runEnd(() -> clawSubsystem.intake(), () -> clawSubsystem.hold(), clawSubsystem),
      Commands.runEnd(() -> clawSubsystem.intake(), () -> clawSubsystem.stop(), clawSubsystem),
      () -> RobotContainer.gamePieceMode == GamePiece.Algae);
  }

  public static Command outtake() {
    return Commands.runEnd(() -> clawSubsystem.outtake(), () -> clawSubsystem.stop(), clawSubsystem);
  }

  public static Command hold() {
    return Commands.run(() -> clawSubsystem.hold(), clawSubsystem);
  }

  public static Command stop() {
    return Commands.run(() -> clawSubsystem.stop(), clawSubsystem);
  }
}
