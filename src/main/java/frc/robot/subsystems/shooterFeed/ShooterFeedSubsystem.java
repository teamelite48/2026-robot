// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterFeed;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.motors.Kraken;
import frc.robot.components.motors.lib.Motor;
import frc.robot.components.sensors.DioPort;

import static frc.robot.subsystems.shooterFeed.ShooterFeedConfig.*;

public class ShooterFeedSubsystem extends SubsystemBase {

  final Motor motor;
  final DigitalInput ballSensor = new DigitalInput(DioPort.BallSensor);

  public ShooterFeedSubsystem() {

    var config = getShooterFeedConfig();

    motor = new Kraken(config);

    initDashboard();
  }

  @Override
  public void periodic() {}

  public void up(){
    motor.setSpeed(UP_SPEED);
  }

  public void down(){
    motor.setSpeed(DOWN_SPEED);
  }

  public void stop(){
    motor.setSpeed(0.0);
  }

  public boolean getBallSensorValue() {
    return ballSensor.get();
  }

  public void initDashboard() {
    var tab = Shuffleboard.getTab("Shooter");

    tab.addDouble("Feed Speed", () -> motor.getVelocity())
      .withPosition(0, 2)
      .withSize(2, 1);
    tab.addBoolean("Ball Sensed", () -> getBallSensorValue())
      .withPosition(2, 2)
      .withSize(2, 1);
  }
}