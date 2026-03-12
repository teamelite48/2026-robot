// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class ShootCommand extends Command {

  TurretSubsystem turretSubsystem = RobotContainer.turretSubsystem;
  VisionSubsystem visionSubsystem = RobotContainer.shooterVisionSubsystem;

  long onSpeedMillis;

  public ShootCommand() {
    // addRequirements(feedSubsystem);
  }

  @Override
  public void initialize() {

    // RobotContainer.isShooting = true;
    // onSpeedMillis = 0;

    // if (RobotContainer.isAimAssistEnabled) {
    //   visionSubsystem.startTracking(VisionTarget.HubApriltag);
    // }
  }

  @Override
  public void execute() {
    turretSubsystem.autoAim();
  }

  @Override
  public void end(boolean interrupted) {
    // RobotContainer.isShooting = false;
    turretSubsystem.stop();
    // visionSubsystem.stopTracking();
  }

  @Override
  public boolean isFinished() {
    // if (RobotContainer.isAutonomous && onSpeedMillis != 0) {
    //   return System.currentTimeMillis() - onSpeedMillis > FeedConfig.TIME_TO_SHOOT_MILLIS;
    // }
    return false;
  }

}