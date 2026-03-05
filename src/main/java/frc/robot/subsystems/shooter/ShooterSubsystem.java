// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.components.motors.Kraken;
import frc.robot.components.motors.lib.Motor;
import frc.robot.lib.LinearInterpolator;
import frc.robot.subsystems.led.LedSubsystem.LedMode;

import static frc.robot.subsystems.shooter.ShooterConfig.*;

import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {

  final Motor leftMotor;
  final Motor rightMotor;

  boolean isShooterOn = false;
  boolean isOnSpeed = false;
  double lastOnSpeedRpm = 0.0;

  boolean isRangeBasedRPMOn = false;

  LinearInterpolator feetToRpmInterpolator = new LinearInterpolator(FEET_TO_RPM_MAP);
  Supplier<Double> feetFromTargetSupplier;

  public ShooterSubsystem(Supplier<Double> feetFromTargetSupplier) {

    this.feetFromTargetSupplier = feetFromTargetSupplier;

    var configLeft = getShooterLeftConfig();
    var configRight = getShooterRightConfig();

    leftMotor = new Kraken(configLeft);  // Leader motor
    rightMotor = new Kraken(configRight);

    rightMotor.follow(leftMotor, true);

    initDashboard();
  }

  @Override
  public void periodic() {

    var currentOnSpeedRpm = RobotContainer.isAimAssistEnabled
      ? feetToRpmInterpolator.calculate(feetFromTargetSupplier.get())
      : ON_SPEED_RPM;

    if (Math.abs(currentOnSpeedRpm - lastOnSpeedRpm) > 5.0) {
      isOnSpeed = false;
    }

    if (isOnSpeed == false && getLeftRpm() > currentOnSpeedRpm) {
      isOnSpeed = true;
      RobotContainer.ledSubsystem.setLedMode(LedMode.Green);
    }
    else if (getLeftRpm() < IDLE_RPM && RobotState.isTeleop()){
      RobotContainer.ledSubsystem.setLedMode(LedMode.Red);
    }

    lastOnSpeedRpm = currentOnSpeedRpm;
  }

  public boolean isShooterOn() {
      return isShooterOn;
  }

  public void setSpeed(double speed) {
    isOnSpeed = false;
    leftMotor.setSpeed(speed);
  }

  public void stop() {
    this.isShooterOn = false;
    this.isOnSpeed = false;
    leftMotor.stop();
  }

  public boolean getIsOnSpeed() {
    return isOnSpeed;
  }

  private double getLeftRpm() {
    return leftMotor.getVelocity() * 60.0;
  }

  private double getRightRpm() {
    return rightMotor.getVelocity() * 60.0;
  }

  public void initDashboard() {
    var tab = Shuffleboard.getTab("Shooter");

    tab.addBoolean("Shooter Enabled", () -> isShooterOn)
      .withPosition(0, 0)
      .withSize(1, 1);

    tab.addDouble("Current RPM Left", () -> getLeftRpm())
      .withPosition(1, 0)
      .withSize(2, 1);

    tab.addDouble("Current RPM Right", () -> getRightRpm())
      .withPosition(3, 0)
      .withSize(2, 1);

    tab.addBoolean("On Speed", () -> getIsOnSpeed())
      .withPosition(0, 1)
      .withSize(1, 1);
  }
}
