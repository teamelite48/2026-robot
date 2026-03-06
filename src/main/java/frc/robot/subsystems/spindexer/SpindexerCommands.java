package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class SpindexerCommands {

  static final SpindexerSubsystem spindexerSubsystem = RobotContainer.spindexerSubsystem;

  public static Command FeedTowardsFeed() {
    return Commands.run(() -> spindexerSubsystem.feedTowardsShooterFeed(), spindexerSubsystem);
  }

  public static Command FeedAwayFromFeed() {
    return Commands.run(() -> spindexerSubsystem.feedAwayFromShooterFeed(), spindexerSubsystem);
  }

  public static Command stop() {
    return Commands.run(() -> spindexerSubsystem.stop(), spindexerSubsystem);
  }
}
