// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.deploy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.components.motors.Kraken;
import frc.robot.components.motors.lib.Motor;
import frc.robot.util.EliteMath;

import static frc.robot.subsystems.deploy.DeployConfig.*;

public class DeploySubsystem extends SubsystemBase {

  public enum ControlMode {
    POSITION, // PID control
    MANUAL    // Raw output
  }

  private final Motor deployMotor;
  private final PIDController pidController;

  private double targetPosition;
  private double currentPosition;
  private double manualSpeed = 0;
  private double outputSpeed = 0;
  private ControlMode currentMode = ControlMode.POSITION;  // TODO: remove this for default set position behavior

  public DeploySubsystem() {

    var configDeploy = getIntakeDeployConfig();
    pidController = new PIDController(configDeploy.pidParameters.P, configDeploy.pidParameters.I, configDeploy.pidParameters.D);
    deployMotor = new Kraken(configDeploy);

    targetPosition = configDeploy.initialPosition;
    currentPosition = configDeploy.initialPosition;

    initDashboard();
  }

  @Override
  public void periodic() {
    currentPosition = getPosition();

    // if (currentMode == ControlMode.POSITION) {
      outputSpeed = pidController.calculate(currentPosition, targetPosition);
      outputSpeed = EliteMath.clamp(outputSpeed, RETRACT_SPEED, EXTEND_SPEED);
    // }
    // else {
    //   outputSpeed = manualSpeed;
    // }

    // --- SHARED SAFETY LIMITS ---
    // Prevent moving past EXTEND_LIMIT
    if (currentPosition >= EXTEND_LIMIT && outputSpeed > 0) {
      outputSpeed = 0;
    }
    // Prevent moving past RETRACT_LIMIT
    if (currentPosition <= RETRACT_LIMIT && outputSpeed < 0) {
      outputSpeed = 0;
    }

    deployMotor.setSpeed(outputSpeed);
  }

  public void manualDrive(double speed) {
    // currentMode = ControlMode.MANUAL;
    this.manualSpeed = speed;
  }

  public double getPosition() {
    return deployMotor.getPosition();
  }

  public void setPosition(double position) {
    // currentMode = ControlMode.POSITION;
    targetPosition = position;
  }

  public void fullExtend() {
    if (RobotContainer.isShooting) {
      setPosition(AGITATE_POSITION);
    }
    else {
      setPosition(EXTEND_LIMIT);
    }

    
  }

  public void fullRetract() {
    setPosition(RETRACT_LIMIT);
  }

  public void agitatePosition() {
    setPosition(AGITATE_POSITION);
  }

  public void extend() {
    deployMotor.setSpeed(EXTEND_SPEED);
  }

  public void retract() {
    deployMotor.setSpeed(-EXTEND_SPEED);
  }

  public void stop() {
    // currentMode = ControlMode.POSITION;
    // manualSpeed = 0;
    targetPosition = getPosition(); // Lock it where it is
    deployMotor.stop();
  }

  public void initDashboard() {
    var tab = Shuffleboard.getTab("Deploy");

    tab.addDouble("Deploy Position", () -> getPosition())
      .withPosition(0, 0)
      .withSize(2, 1);

    tab.addDouble("Deploy Target", () -> targetPosition)
      .withPosition(2, 0)
      .withSize(2, 1);

    // tab.addDouble("Extend Limit", () -> EXTEND_LIMIT)
    //   .withPosition(0, 1)
    //   .withSize(2, 1);

    // tab.addDouble("Retract Limit", () -> RETRACT_LIMIT)
    //   .withPosition(2, 2)
    //   .withSize(2, 1);

    // tab.addString("Control Mode", () -> currentMode.toString())
    //   .withPosition(4, 1);
  }
}
