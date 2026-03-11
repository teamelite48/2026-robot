// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class ShooterCommands {

    static final ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;

    public static Command stop() {
        return Commands.run(() -> shooterSubsystem.setOff(), shooterSubsystem);
    }

    public static Command idleShooter() {
        return Commands.run(() -> shooterSubsystem.isIdling(), shooterSubsystem);
    }
}
