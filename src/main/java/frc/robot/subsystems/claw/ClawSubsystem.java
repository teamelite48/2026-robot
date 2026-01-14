package frc.robot.subsystems.claw;

import static frc.robot.subsystems.claw.ClawConfig.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotConfig.GamePiece;
import frc.robot.components.motors.Kraken;
import frc.robot.components.motors.lib.Motor;

public class ClawSubsystem extends SubsystemBase {

  final Motor motor;

  public ClawSubsystem() {
    motor = new Kraken(getMotorConfig());
  }

  public void periodic() {}

  public void intake() {
    if (RobotContainer.gamePieceMode == GamePiece.Coral){
      motor.setSpeed(CORAL_INTAKE_SPEED);
    }
    else {
      motor.setSpeed(ALGAE_INTAKE_SPEED);
    }
  }

  public void hold() {
    motor.setSpeed(HOLD_SPEED);
  }

  public void outtake() {

    if (RobotContainer.gamePieceMode == GamePiece.Algae) {
      motor.setSpeed(ALGAE_OUTAKE_SPEED);
    }
    else {
      motor.setSpeed(CORAL_OUTTAKE_SPEED);
    }
  }

  public void stop() {
    motor.stop();
  }
}
