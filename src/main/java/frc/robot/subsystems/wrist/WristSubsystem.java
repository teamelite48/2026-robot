package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotConfig.GamePiece;
import frc.robot.components.motors.Kraken;
import frc.robot.components.motors.lib.Motor;
import frc.robot.util.EliteMath;

import static frc.robot.subsystems.wrist.WristConfig.*;

public class WristSubsystem extends SubsystemBase {

  final Motor rightMotor;
  final Motor leftMotor;

  double leftMotorCurrentPosition = INITIAL_DEGREES;
  double rightMotorCurrentPosition = INITIAL_DEGREES;

  double leftMotorTargetPosition = leftMotorCurrentPosition;
  double rightMotorTargetPosition = rightMotorCurrentPosition;

  final PIDController leftPidController;
  final PIDController rightPidController;

  public WristSubsystem() {

    var leftMotorConfig = getLeftMotorConfig();
    var rightMotorConfig = getRightMotorConfig();

    leftMotor = new Kraken(leftMotorConfig);
    rightMotor = new Kraken(rightMotorConfig);

    leftPidController = new PIDController(WristConfig.PID_PARAMETERS.P, WristConfig.PID_PARAMETERS.I, WristConfig.PID_PARAMETERS.D);
    rightPidController = new PIDController(WristConfig.PID_PARAMETERS.P, WristConfig.PID_PARAMETERS.I, WristConfig.PID_PARAMETERS.D);

    initDashboard();
  }

  @Override
  public void periodic() {

    leftMotorCurrentPosition = leftMotor.getPosition();
    rightMotorCurrentPosition = rightMotor.getPosition();

    var leftSpeed = leftPidController.calculate(leftMotorCurrentPosition, leftMotorTargetPosition);
    var rightSpeed = rightPidController.calculate(rightMotorCurrentPosition, rightMotorTargetPosition);

    if (RobotContainer.gamePieceMode == GamePiece.Coral){
      leftMotor.setSpeed(EliteMath.clamp(leftSpeed, WristConfig.MAX_SPEED_CORAL));
      rightMotor.setSpeed(EliteMath.clamp(rightSpeed, WristConfig.MAX_SPEED_CORAL));
    }
    else{
      leftMotor.setSpeed(EliteMath.clamp(leftSpeed, WristConfig.MAX_SPEED_ALGAE));
      rightMotor.setSpeed(EliteMath.clamp(rightSpeed, WristConfig.MAX_SPEED_ALGAE));
    }
  }

  public void tiltUp() {

    var newPosition = new WristPosition(
      getTargetRotationDegrees(),
      getTargetTiltDegrees() - WristConfig.MANUAL_TARGET_MODIFIER
    );

    setPosition(newPosition);
  }

  public void tiltDown() {

    var newPosition = new WristPosition(
      getTargetRotationDegrees(),
      getTargetTiltDegrees() + WristConfig.MANUAL_TARGET_MODIFIER
    );

    setPosition(newPosition);
  }

  public void rotateClockwise() {

    var newPosition = new WristPosition(
      getTargetRotationDegrees() + WristConfig.MANUAL_TARGET_MODIFIER,
      getTargetTiltDegrees()
    );

    setPosition(newPosition);
  }

  public void rotateCounterClockwise() {

    var newPosition = new WristPosition(
      getTargetRotationDegrees() - WristConfig.MANUAL_TARGET_MODIFIER,
      getTargetTiltDegrees()
    );

    setPosition(newPosition);
  }

  public void setPosition(WristPosition wristPosition) {

    leftMotorTargetPosition = toLeftMotorPosition(wristPosition);
    rightMotorTargetPosition = toRightMotorPosition(wristPosition);
  }

  // TODO: remove once it works
  // public WristPosition getWristPosition(double rotation, double tilt) {
  //   return new WristPosition(rotation - tilt, -(rotation + tilt));
  // }

  public double toLeftMotorPosition(WristPosition wristPosition) {
    return wristPosition.rotationDegrees - wristPosition.tiltDegrees;
  }

  public double toRightMotorPosition(WristPosition wristPosition) {
    return -(wristPosition.rotationDegrees + wristPosition.tiltDegrees);
  }

  public double getCurrentTiltDegrees() {
    return -(leftMotor.getPosition() + rightMotor.getPosition()) / 2.0;
  }

  public double getCurrentRotationDegrees() {
    return (leftMotor.getPosition() - rightMotor.getPosition()) / 2.0;
  }

  public double getTargetTiltDegrees() {
    return -(leftMotorTargetPosition + rightMotorTargetPosition) / 2.0;
  }

  public double getTargetRotationDegrees() {
    return (leftMotorTargetPosition - rightMotorTargetPosition) / 2.0;
  }

  public boolean isAtTargetPosition() {
    return Math.abs(leftMotorTargetPosition - leftMotorCurrentPosition) <= WristConfig.TARGET_THRESHOLD
      && Math.abs(rightMotorTargetPosition - rightMotorCurrentPosition) <= WristConfig.TARGET_THRESHOLD;
  }

  private void initDashboard() {
    var wristTab = Shuffleboard.getTab("Wrist");

    wristTab.addDouble("Left Position", () -> leftMotor.getPosition())
      .withPosition(0, 0)
      .withSize(2, 1);

    wristTab.addDouble("Right Position", () -> rightMotor.getPosition())
      .withPosition(2, 0)
      .withSize(2, 1);

    wristTab.addDouble("Left Calculated Position", () -> toLeftMotorPosition(new WristPosition(getCurrentRotationDegrees(), getCurrentTiltDegrees())))
      .withPosition(0, 1)
      .withSize(2, 1);

    wristTab.addDouble("Right Calculated Position", () -> toRightMotorPosition(new WristPosition(getCurrentRotationDegrees(), getCurrentTiltDegrees())))
      .withPosition(2, 1)
      .withSize(2, 1);

    wristTab.addDouble("Left Target Position", () -> leftMotorTargetPosition)
      .withPosition(0, 2)
      .withSize(2, 1);

    wristTab.addDouble("Right Target Position", () -> rightMotorTargetPosition)
      .withPosition(2, 2)
      .withSize(2, 1);

    wristTab.addDouble("Tilt Degrees", () -> getCurrentTiltDegrees())
      .withPosition(0, 3)
      .withSize(2, 1);

    wristTab.addDouble("Rotation Degrees", () -> getCurrentRotationDegrees())
      .withPosition(2, 3)
      .withSize(2, 1);

    wristTab.addDouble("Target Tilt Degrees", () -> getTargetTiltDegrees())
      .withPosition(0, 4)
      .withSize(2, 1);

    wristTab.addDouble("Target Rotation Degrees", () -> getTargetRotationDegrees())
      .withPosition(2, 4)
      .withSize(2, 1);
  }
}