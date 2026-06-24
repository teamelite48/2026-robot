
//TRAVIS
/*

package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConfig.HOME_POSITION;
import static frc.robot.subsystems.turret.TurretConfig.degreesAtCenter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.commands.MoveTurretToDegrees;

public class TurretCommands {

  static final TurretSubsystem turretSubsystem = RobotContainer.turretSubsystem;

  public static Command RotateTurretClockwise() {
    return Commands.run(() -> turretSubsystem.rotateClockwise(), turretSubsystem);
  }

  // @Override
  // public void initialize() {}

  public static Command RotateTurretCounterClockwise() {
    return Commands.run(() -> turretSubsystem.rotateCounterClockwise(), turretSubsystem);
  }

  public static Command moveToSafeState() {
    return new MoveTurretToDegrees(HOME_POSITION);
  }

  public static Command moveTo90() {
    return new MoveTurretToDegrees(degreesAtCenter);
  }

  public static Command stop() {
    return Commands.run(() -> turretSubsystem.stop(), turretSubsystem);
  }

  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}

*/
