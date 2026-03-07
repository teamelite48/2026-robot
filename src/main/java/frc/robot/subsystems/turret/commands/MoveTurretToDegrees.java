// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.TurretConfig;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.CoolDownTimer;

public class MoveTurretToDegrees extends Command {

  TurretSubsystem turretSubsystem = RobotContainer.turretSubsystem;
  CoolDownTimer coolDown = new CoolDownTimer(TurretConfig.moveCoolDown);
  double degrees;

  public MoveTurretToDegrees(double degrees) {
    addRequirements(turretSubsystem);

    this.degrees = degrees;
  }

  @Override
  public void initialize() {
    coolDown.start();
  }

  @Override
  public void execute() {
    turretSubsystem.turnAutoAimOff();
    turretSubsystem.moveToDegrees(degrees);
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(turretSubsystem.getPositionInDegrees() - degrees) < TurretConfig.moveWithinDegrees
      || coolDown.isCool();
  }
}
