// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.motors.Kraken;
import frc.robot.components.motors.lib.Motor;
import frc.robot.lib.LinearInterpolator;

import static frc.robot.subsystems.shooter.ShooterConfig.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class ShooterSubsystem extends SubsystemBase {

  final Motor leftMotor;
  final Motor rightMotor;

  // final SimpleMotorFeedforward rearFeedForward = ShooterConfig.rearMotorFeedForward;

  boolean isShooterOn = false;
  double targetRPM = MEDIUM_RPM;
  boolean isRangeBasedRPMOn = false;
  double distanceToTargetInFeet = 0.0;

  // LinearInterpolator distanceToRPMInterpolator = new LinearInterpolator(DISTANCE_TO_RPM_MAP);

  public ShooterSubsystem() {
    var configLeft = getShooterLeftConfig();
    var configRight = getShooterRightConfig();

    leftMotor = new Kraken(configLeft);
    rightMotor = new Kraken(configRight);
  }

  @Override
  public void periodic() {
    // double angleToGoalInDegrees = ShooterConfig.limelightAngleInDegrees + (ty.getDouble(0.0));
    // double angleToGoalInRadians = Math.toRadians(angleToGoalInDegrees);

    // distanceToTargetInFeet = Math.min(ShooterConfig.limelightToVisionTargetInFeet / Math.tan(angleToGoalInRadians), ShooterConfig.maxDistanceInFeet);

    if (isShooterOn == false) {
      leftMotor.setSpeed(0.0);
      rightMotor.setSpeed(0.0);
    }
    else {

      if(isRangeBasedRPMOn){
        // targetRPM = distanceToRPMInterpolator.calculate(distanceToTargetInFeet);
      }

      setLeftMotor(targetRPM);
      setRightMotor(targetRPM * 2.5);
    }
  }

  private void setLeftMotor(double targetRPM) {
    // double frontCurrentRPM = -1 * (leftMotor.getSensorCollection().getIntegratedSensorVelocity() * 600) / 2048;
    // leftMotor.setVoltage(leftFeedForward.calculate(targetRPM / 60) + leftPIDController.calculate(leftCurrentRPM, targetRPM));

    // rightMotor.setControl(new Follower(leftMotor.getCanId(), MotorAlignmentValue.Opposed));
  }

  private void setRightMotor(double targetRPM) {
    // double rearCurrentRPM = -1 * (rightMotor.getSensorCollection().getIntegratedSensorVelocity() * 600) / 2048;

    // rightMotor.setVoltage(rightFeedForward.calculate(targetRPM / 60) + rightPIDController.calculate(rightCurrentRPM, targetRPM));

    // SmartDashboard.putNumber("Current Rear RPM", rearCurrentRPM * -1);
  }

  public void toggleShooter() {
    isShooterOn = !isShooterOn;
  }

  public void shooterOn() {
    isShooterOn = true;
  }

  public void shooterOff() {
    isRangeBasedRPMOn = false;
    isShooterOn = false;
  }

  public void setLowSpeed() {
    isRangeBasedRPMOn = false;
    targetRPM = LOW_RPM;
  }

  public void setMediumSpeed() {
    isRangeBasedRPMOn = false;
    targetRPM = MEDIUM_RPM;
  }

  public void turnRangeBasedRPMOn() {
    isRangeBasedRPMOn = true;
  }

  public void turnRangeBasedRPMOff() {
    isRangeBasedRPMOn = false;
  }

  public boolean isShooterOn() {
      return isShooterOn;
  }

  public void bumpRpmUp() {
    targetRPM += RPM_BUMP;
  }

  public void bumpRpmDown() {
    targetRPM -= RPM_BUMP;
  }

  public double getTargetRpm() {
    return targetRPM;
  }

  public double getDistanceToTarget() {
    return distanceToTargetInFeet;
  }

  public void initDashboard() {
    var tab = Shuffleboard.getTab("Shooter");

    tab.addDouble("Target RPM", () -> getTargetRpm())
      .withPosition(0, 0)
      .withSize(2, 1);

    tab.addBoolean("Shooter On", () -> isShooterOn)
      .withPosition(2, 0)
      .withSize(1, 1);

    tab.addDouble("Distance to Target", () -> getDistanceToTarget())
      .withPosition(3, 0)
      .withSize(2, 1);

    tab.addDouble("Current RPM Left", () -> getTargetRpm())
      .withPosition(0, 1)
      .withSize(2, 1);

    tab.addDouble("Current RPM Right", () -> getTargetRpm())
      .withPosition(2, 1)
      .withSize(2, 1);
  }
}
