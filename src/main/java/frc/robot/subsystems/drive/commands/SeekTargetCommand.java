package frc.robot.subsystems.drive.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.CoolDownTimer;

public class SeekTargetCommand extends Command {

  DriveSubsystem driveSubsystem = RobotContainer.driveSubsystem;
  VisionSubsystem visionSubsystem;
  CoolDownTimer coolDownTimer = new CoolDownTimer(1000);

  public SeekTargetCommand() {
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    // visionSubsystem.startTracking(VisionTarget.HubApriltag);
  }

  @Override
  public void execute() {
    var xOffset = visionSubsystem.getXOffsetDegrees();

    driveSubsystem.driveRobotRelative(-((xOffset * 0.01) + (Math.signum(xOffset) * 0.01)), 0);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {

    if (Math.abs(visionSubsystem.getXOffsetDegrees()) > 1.0) {
      coolDownTimer.start();
    }

    // visionSubsystem.stopTracking();

    return coolDownTimer.isCool();
  }
}