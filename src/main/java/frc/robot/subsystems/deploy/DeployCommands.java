// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.deploy;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class DeployCommands {
    static final DeploySubsystem deploySubsystem = RobotContainer.deploySubsystem;

    public static Command extend() {
        return Commands.run(() -> deploySubsystem.extend(), deploySubsystem);
    }

    public static Command retract() {
        return Commands.run(() -> deploySubsystem.retract(), deploySubsystem);
    }

    public static Command stop() {
        return Commands.run(() -> deploySubsystem.stop(), deploySubsystem);
    }

    // TODO: Add commands for switching modes & manual setPosition
}
