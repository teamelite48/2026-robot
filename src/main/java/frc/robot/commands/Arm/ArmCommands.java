package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConfig.GamePiece;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorCommands;
import frc.robot.subsystems.shoulder.ShoulderCommands;
import frc.robot.subsystems.wrist.WristCommands;
import frc.robot.subsystems.wrist.WristConfig;
import frc.robot.subsystems.wrist.WristPosition;

public class ArmCommands {

  public static Command HomeArm() {

    var algae = GetIntoPosition(-90, 45, 90, 0, false);
    var coral = GetIntoPosition(0, 0, 90, 0, false);

    return GetIntoPositionByMode(algae, coral);
  }

  public static Command FloorPickup() {

    var algae = GetIntoPosition(-90, 76, -14, 6, false);
    var coral = GetIntoPosition(-90, 54, -24, 10.5, false);

    return GetIntoPositionByMode(algae, coral);
  }

  public static Command LowAlgaeAndStationPickup() {

    var algae = GetIntoPosition(0, 152, 123, 0, false);
    var coral = GetIntoPosition(-90, 48, 60, 0, false);

    return GetIntoPositionByMode(algae, coral);
  }

  public static Command L3Pickup() {

    var algae = GetIntoPosition(0, 158, 110, 13.5, false);
    var coral = Commands.none();

    return GetIntoPositionByMode(algae, coral);
  }

  public static Command L1andProcessorScore() {

    var algae = GetIntoPosition(-90, 119, -14.5, 0, false);
    var coral = GetIntoPosition(-90, 106, 168, 0, false);

    return GetIntoPositionByMode(algae, coral);
  }

  public static Command L2Score() {

    var algae = Commands.none();
    var coral = GetIntoPosition(0, 110, 131, 0, true);

    return GetIntoPositionByMode(algae, coral);
  }

  public static Command L3Score() {

    var algae = Commands.none();
    var coral = GetIntoPosition(0, 113, 112, 11, true);

    return GetIntoPositionByMode(algae, coral);
  }

  public static Command L4andBargeScore() {

    var algae = GetIntoPosition(-90, 45, 90, 46, false);
    var coral = GetIntoPosition(0, 138, 95, 36, true);

    return GetIntoPositionByMode(algae, coral);
  }

  public static Command HangingPosition() {

    var algae = GetIntoPosition(-90, 57, 118, 0, false);
    var coral = GetIntoPosition(-90, 57, 118, 0, false);

    return GetIntoPositionByMode(algae, coral);
  }

  private static Command GetIntoPositionByMode(Command algaeCommand, Command coralCommand) {
    return Commands.either(
      algaeCommand,
      coralCommand,
      () -> RobotContainer.gamePieceMode == GamePiece.Algae);
  }

  private static Command GetIntoPosition(double wristRotationDegrees, double wristTiltDegress, double shoulderAngle, double elevatorPosition, boolean isWristFlippable) {

    return Commands.parallel(
      Commands.sequence(
        WristCommands.goHome(),
        WristCommands.setPosition(new WristPosition(wristRotationDegrees, WristConfig.HOME_TILT_DEGREES)),
        WristCommands.setPosition(new WristPosition(wristRotationDegrees, wristTiltDegress)),
        Commands.runOnce(() -> RobotContainer.isWristFlippable = isWristFlippable)
      ),
      ShoulderCommands.setAngle(shoulderAngle),
      ElevatorCommands.setHeight(elevatorPosition)
    );
  }
}
