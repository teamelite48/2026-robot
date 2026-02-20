// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class IntakeCommands {
    static final IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;

    public static Command extend() {
        return Commands.run(() -> intakeSubsystem.extend(), intakeSubsystem);
    }

    public static Command retract() {
        return Commands.run(() -> intakeSubsystem.retract(), intakeSubsystem);
    }

    public static Command stop() {
        return Commands.run(() -> intakeSubsystem.stopIntake(), intakeSubsystem);
    }
}
