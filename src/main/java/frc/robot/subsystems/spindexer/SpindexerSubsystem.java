// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.motors.Kraken;
import frc.robot.components.motors.lib.Motor;

import static frc.robot.subsystems.spindexer.SpindexerConfig.*;

public class SpindexerSubsystem extends SubsystemBase {

  final Motor motor;

  public SpindexerSubsystem() {

    var config = getSpindexerConfig();

    motor = new Kraken(config);

    initDashboard();
  }

  @Override
  public void periodic() {}

  public void feedTowardsShooterFeed() {
    motor.setSpeed(FEED_SPEED);
  }

  public void feedAwayFromShooterFeed() {
    motor.setSpeed(REVERSE_SPEED);
  }

  public void stop() {
    motor.setSpeed(0.0);
  }

  public void initDashboard() {
    var tab = Shuffleboard.getTab("Spindexer");

    tab.addDouble("Speed", () -> motor.getVelocity())
      .withPosition(0, 0)
      .withSize(2, 1);
  }
}
