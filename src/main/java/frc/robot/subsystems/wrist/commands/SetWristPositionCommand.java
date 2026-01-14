
package frc.robot.subsystems.wrist.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.wrist.WristPosition;
import frc.robot.subsystems.wrist.WristSubsystem;

public class SetWristPositionCommand extends Command {

  public WristSubsystem wristSubsystem = RobotContainer.wristSubsystem;

  WristPosition targetPosition;

  public SetWristPositionCommand(WristPosition targetPosition) {
    this.targetPosition = targetPosition;
    addRequirements(wristSubsystem);
  }

  @Override
  public void initialize() {
    wristSubsystem.setPosition(targetPosition);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return wristSubsystem.isAtTargetPosition();
  }
}
