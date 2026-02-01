package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.commands.DriveCommand;
import frc.robot.subsystems.drive.commands.SeekTargetCommand;

public class DriveCommands {

  static DriveSubsystem driveSubsystem = RobotContainer.driveSubsystem;

  public static Command drive(Supplier<Translation2d> translationSupplier, Supplier<Translation2d> rotationSupplier) {
    return new DriveCommand(translationSupplier, rotationSupplier);
  }

  public static Command seekTarget() {
    return new SeekTargetCommand();
  }

  public static Command strafeLeft() {
    return Commands.run(() -> driveSubsystem.driveRobotRelative(driveSubsystem.getStrafeSpeed(), 0), driveSubsystem);
  }

  public static Command strafeRight() {
    return Commands.run(() -> driveSubsystem.driveRobotRelative(-driveSubsystem.getStrafeSpeed(), 0), driveSubsystem);
  }
}