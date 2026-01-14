package frc.robot.subsystems.shoulder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.encoders.absolute.DutyCycleCoder;
import frc.robot.components.encoders.absolute.lib.AbsoluteEncoder;
import frc.robot.components.motors.Neo550;
import frc.robot.components.motors.lib.Motor;
import frc.robot.util.EliteMath;

import static frc.robot.subsystems.shoulder.ShoulderConfig.*;

public class ShoulderSubsystem extends SubsystemBase {

  final Motor motor;
  // final AbsoluteEncoder absoluteEncoder;

  final PIDController pidController;

  boolean isInitialized = false;
  double targetAngle;
  double currentAngle;

  public ShoulderSubsystem() {

    // absoluteEncoder = new DutyCycleCoder(getAbsoluteEncoderConfig());    //enable if have abs encoder
    // currentAngle = absoluteEncoder.getPosition();                        //enable if have abs encoder

    currentAngle = ShoulderConfig.INITIAL_POSITION;                         //disable if abs encoder is present
    targetAngle = currentAngle;

    motor = new Neo550(getMotorConfig(currentAngle));
    pidController = new PIDController(ShoulderConfig.PID_PARAMETERS.P, ShoulderConfig.PID_PARAMETERS.I, ShoulderConfig.PID_PARAMETERS.D);
    initDashboard();
  }

  public void periodic() {

    if (isInitialized == false) {
      if (Math.abs(motor.getPosition() - currentAngle) > 0.001) {
        motor.setInitialPosition();
        return;
      }
      else {
        isInitialized = true;
      }
    }

    currentAngle = motor.getPosition();

    var nextOutput = pidController.calculate(currentAngle, targetAngle);

    var speed = EliteMath.clamp(nextOutput, ShoulderConfig.MAX_SPEED);

    motor.setSpeed(speed);
  }

  public void tiltUp() {
    setAngle(targetAngle += ShoulderConfig.MANUAL_TARGET_MODIFIER);
  }

  public void tiltDown() {
    setAngle(targetAngle -= ShoulderConfig.MANUAL_TARGET_MODIFIER);
  }

  public void setAngle(double degrees) {
    targetAngle = degrees;
  }

  public boolean isAtTargetAngle() {
    return Math.abs(targetAngle - currentAngle) <= ShoulderConfig.TARGET_THRESHOLD;
  }

  public double getAngle() {
    return motor.getPosition();
  }

  public void stop() {
    motor.stop();
  }

  private void initDashboard() {
    var shoulderTab = Shuffleboard.getTab("Shoulder");

    // shoulderTab.addDouble("Absolute Angle", () -> absoluteEncoder.getPosition())
    //   .withPosition(0, 0)
    //   .withSize(2, 1);

    shoulderTab.addDouble("Relative Angle", () -> motor.getPosition())
      .withPosition(3, 0)
      .withSize(2, 1);

    shoulderTab.addDouble("Target Angle", () -> targetAngle)
      .withPosition(0, 3)
      .withSize(2, 1);

    shoulderTab.addBoolean("Is Initialized", () -> isInitialized)
      .withPosition(0, 2)
      .withSize(1, 1);
  }
}
