package frc.robot.subsystems.wrist.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.wrist.WristConfig;
import frc.robot.subsystems.wrist.WristPosition;
import frc.robot.subsystems.wrist.WristSubsystem;

public class SendWristHomeCommand extends Command {

  WristSubsystem wristSubsystem = RobotContainer.wristSubsystem;

  public SendWristHomeCommand() {
    addRequirements(wristSubsystem);
  }

  @Override
  public void initialize() {
    wristSubsystem.setPosition(new WristPosition(wristSubsystem.getCurrentRotationDegrees(), WristConfig.HOME_TILT_DEGREES));
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
