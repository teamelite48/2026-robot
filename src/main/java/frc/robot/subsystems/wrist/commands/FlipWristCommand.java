package frc.robot.subsystems.wrist.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.wrist.WristPosition;
import frc.robot.subsystems.wrist.WristSubsystem;

public class FlipWristCommand extends Command {

  WristSubsystem wristSubsystem = RobotContainer.wristSubsystem;
  Supplier<WristPosition> lastWristPositionSupplier;

  public FlipWristCommand(Supplier<WristPosition> lastWristPositionSupplier) {
    this.lastWristPositionSupplier = lastWristPositionSupplier;
    addRequirements(wristSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    var lastWristPosition = lastWristPositionSupplier.get();

    var newRotation = lastWristPosition.rotationDegrees > 90.0
      ? 0
      : 180;

    wristSubsystem.setPosition(new WristPosition(newRotation, lastWristPosition.tiltDegrees));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
