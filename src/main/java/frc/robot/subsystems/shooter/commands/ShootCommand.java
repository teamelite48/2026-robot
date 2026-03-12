// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.shooter.commands;

// import static frc.robot.subsystems.shooter.ShooterConfig.FEET_TO_RPM_MAP;
// import static frc.robot.subsystems.shooter.ShooterConfig.MEDIUM_RPM;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.shooter.ShooterSubsystem;
// import frc.robot.subsystems.shooterFeed.ShooterFeedSubsystem;
// import frc.robot.subsystems.spindexer.SpindexerSubsystem;
// import frc.robot.subsystems.turret.TurretSubsystem;
// import frc.robot.subsystems.vision.VisionSubsystem;
// import frc.robot.subsystems.vision.VisionSubsystem.VisionTarget;

// public class ShootCommand extends Command {

//   ShooterFeedSubsystem shooterFeedSubsystem = RobotContainer.shooterFeedSubsystem;
//   ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;
//   SpindexerSubsystem spindexerSubsystem = RobotContainer.spindexerSubsystem;
//   TurretSubsystem turretSubsystem = RobotContainer.turretSubsystem;
//   VisionSubsystem shooterVisionSubsystem = RobotContainer.shooterVisionSubsystem;

//   long onSpeedMillis;

//   public ShootCommand() {
//     addRequirements(shooterFeedSubsystem, shooterSubsystem, spindexerSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//     RobotContainer.isShooting = true;
//     onSpeedMillis = 0;

//     if (RobotContainer.isAimAssistEnabled) {
//       shooterVisionSubsystem.startTracking(VisionTarget.HubApriltag);
//     }
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (feedAssist()) {
//       if (onSpeedMillis == 0) {
//         onSpeedMillis = System.currentTimeMillis();
//       }

//       shooterSubsystem.setShooterRPM(shooterSubsystem.getTargetRPM());
//       shooterFeedSubsystem.feedTowardsShooter();
//       spindexerSubsystem.feedTowardsShooterFeed();
//     }
//     else if (noFeedAssist()) {
//       shooterSubsystem.setShooterRPM(MEDIUM_RPM);
//       shooterFeedSubsystem.feedTowardsShooter();
//       spindexerSubsystem.feedTowardsShooterFeed();
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     RobotContainer.isShooting = false;
//     shooterFeedSubsystem.stop();
//     // shooterVisionSubsystem.stopTracking();
//     shooterSubsystem.idleShooter();
//     spindexerSubsystem.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     if (RobotContainer.isAutonomous && onSpeedMillis != 0) {
//       // return System.currentTimeMillis() - onSpeedMillis > ShooterFeedConfig.TIME_TO_SHOOT_MILLIS;
//     }
//     return false;
//   }

//   private boolean feedAssist() {
//     return !noFeedAssist()
//       && RobotContainer.isAimAssistEnabled
//       && shooterSubsystem.getIsOnSpeed()
//       && shooterVisionSubsystem.hasTargetWithinParameters();
//   }

//   private boolean noFeedAssist() {
//     return RobotContainer.isAimAssistEnabled == false;
//   }
// }


package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.ShooterConfig;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterFeed.ShooterFeedSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class ShootCommand extends Command {

  private final ShooterFeedSubsystem shooterFeedSubsystem = RobotContainer.shooterFeedSubsystem;
  private final ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;
  private final SpindexerSubsystem spindexerSubsystem = RobotContainer.spindexerSubsystem;
  private final TurretSubsystem turretSubsystem = RobotContainer.turretSubsystem;
  private final VisionSubsystem shooterVisionSubsystem = RobotContainer.shooterVisionSubsystem;

  private final ShooterConfig.ShooterPreset preset;

  public ShootCommand(ShooterConfig.ShooterPreset preset) {
    this.preset = preset;
    addRequirements(shooterFeedSubsystem, spindexerSubsystem);
  }

  @Override
  public void initialize() {
    RobotContainer.isShooting = true;

    shooterSubsystem.useAimAssistOrPreset(preset);

    // if (RobotContainer.isAimAssistEnabled) {
    //   shooterVisionSubsystem.startTracking(VisionTarget.HubApriltag);
    // }
  }

  @Override
  public void execute() {
    boolean readyToFeed;

    if (RobotContainer.isAimAssistEnabled) {
      readyToFeed =
          shooterSubsystem.getIsOnSpeed();
          //&& shooterVisionSubsystem.hasTargetWithinParameters();
    } else {
      readyToFeed = shooterSubsystem.getIsOnSpeed();
    }

    if (readyToFeed) {
      shooterFeedSubsystem.feedTowardsShooter();
      spindexerSubsystem.feedTowardsShooterFeed();
    } else {
      shooterFeedSubsystem.stop();
      spindexerSubsystem.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.isShooting = false;
    shooterFeedSubsystem.stop();
    spindexerSubsystem.stop();
    shooterSubsystem.setOff();
    // shooterVisionSubsystem.stopTracking();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}