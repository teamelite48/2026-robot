// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climbAssist;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.servos.CanRevServo;

import static frc.robot.subsystems.climbAssist.ClimbAssistConfig.*;

public class ClimbAssistSubsystem extends SubsystemBase {

  final CanRevServo servo;

  boolean isClimberLocked = false;

  public ClimbAssistSubsystem() {
    var config = getServoConfig();
    servo = new CanRevServo(config);

    initDashboard();
  }

  @Override
  public void periodic() {}

  public void lock() {
    servo.setPosition(LOCK_VALUE);
    isClimberLocked = true;
  }

  public void unlock() {
    servo.setPosition(UNLOCK_VALUE);
    isClimberLocked = false;
  }

  public boolean getIsClimberLocked() {
    return isClimberLocked;
  }

  public double getClimbAssistPosition() {
    return servo.getPosition();
  }

  public void initDashboard() {
    var tab = Shuffleboard.getTab("Climber");

    tab.addDouble("Assist Position", () -> getClimbAssistPosition())
      .withPosition(0, 1)
      .withSize(2, 1);

      tab.addBoolean("Is Assist Locked", () -> getIsClimberLocked())
        .withPosition(2, 1)
        .withSize(1, 1);

      tab.addBoolean("Is Servo Enabled", () -> servo.getIsServoEnabled())
        .withPosition(4, 1)
        .withSize(1, 1);
  }
}
