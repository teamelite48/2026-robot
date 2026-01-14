package frc.robot.subsystems.shoulder.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;

public class SetShoulderAngleCommand extends Command {

  ShoulderSubsystem shoulderSubsystem = RobotContainer.shoulderSubsystem;

  double targetAngle;

  public SetShoulderAngleCommand(double degrees) {
    this.targetAngle = degrees;
    addRequirements(shoulderSubsystem);
  }

  @Override
  public void initialize() {
    shoulderSubsystem.setAngle(targetAngle);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return shoulderSubsystem.isAtTargetAngle();
  }
}
