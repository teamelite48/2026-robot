// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterFeed.ShooterFeedConfig;
import frc.robot.subsystems.shooterFeed.ShooterFeedSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.VisionTarget;

public class ShootCommand extends Command {

  ShooterFeedSubsystem shooterFeedSubsystem = RobotContainer.shooterFeedSubsystem;
  ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;
  TurretSubsystem turretSubsystem = RobotContainer.turretSubsystem;
  VisionSubsystem shooterVisionSubsystem = RobotContainer.shooterVisionSubsystem;

  long onSpeedMillis;

  public ShootCommand() {
    addRequirements(shooterFeedSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    RobotContainer.isShooting = true;
    onSpeedMillis = 0;

    if (RobotContainer.isAimAssistEnabled) {
      shooterVisionSubsystem.startTracking(VisionTarget.HubApriltag);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (feedAssist()) {
      if (onSpeedMillis == 0) {
        onSpeedMillis = System.currentTimeMillis();
      }

      shooterFeedSubsystem.feedTowardsShooter();
    }
    else if (noFeedAssist()) {
      shooterFeedSubsystem.feedTowardsShooter();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.isShooting = false;
    shooterFeedSubsystem.stop();
    shooterVisionSubsystem.stopTracking();
    shooterSubsystem.idle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotContainer.isAutonomous && onSpeedMillis != 0) {
      // return System.currentTimeMillis() - onSpeedMillis > ShooterFeedConfig.TIME_TO_SHOOT_MILLIS;
    }
    return false;
  }

  private boolean feedAssist() {
    return !noFeedAssist()
      && RobotContainer.isAimAssistEnabled
      && shooterSubsystem.getIsOnSpeed()
      && shooterVisionSubsystem.hasTarget();
  }

  private boolean noFeedAssist() {
    return RobotContainer.isAimAssistEnabled == false;
  }
}
