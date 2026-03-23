// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.deploy;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class DeployCommands {
    static final DeploySubsystem deploySubsystem = RobotContainer.deploySubsystem;

    public static Command extend() {
        return Commands.run(() -> deploySubsystem.extend(), deploySubsystem);
    }

    public static Command retract() {
        return Commands.run(() -> deploySubsystem.retract(), deploySubsystem);
    }

    public static Command fullExtend() {
        return Commands.run(() -> deploySubsystem.fullExtend(), deploySubsystem);
    }

    public static Command agitatePosition() {
        return Commands.run(() -> deploySubsystem.agitatePosition(), deploySubsystem);
    }

    // Never full retract unless turret is in safe place
    public static Command fullRetract() {
        return Commands.run(() -> deploySubsystem.fullRetract(), deploySubsystem);
    }

    public static Command setToHome() {
        return Commands.run(() -> deploySubsystem.setPosition(DeployConfig.HOME_POSITION), deploySubsystem);
    }

    public static Command stop() {
        return Commands.run(() -> deploySubsystem.stop(), deploySubsystem);
    }

    public static Command agitate() {
        return Commands.sequence(
            Commands.deadline(new WaitCommand(0.5), agitatePosition()),
            new WaitCommand(0.25),
            Commands.deadline(new WaitCommand(0.5), setToHome()),
            new WaitCommand(0.25)
        ).repeatedly();
    }
}
