package frc.robot.subsystems.shooterFeed;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class ShooterFeedCommands {

  static final ShooterFeedSubsystem shooterFeedSubsystem = RobotContainer.shooterFeedSubsystem;

  public static Command FeedTowardsShooter() {
    return Commands.run(() -> shooterFeedSubsystem.feedTowardsShooter(), shooterFeedSubsystem);
  }

  public static Command FeedAwayFromShooter() {
    return Commands.run(() -> shooterFeedSubsystem.feedAwayFromShooter(), shooterFeedSubsystem);
  }

  public static Command stop() {
    return Commands.run(() -> shooterFeedSubsystem.stop(), shooterFeedSubsystem);
  }

  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}
