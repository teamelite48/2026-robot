// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.motors.Kraken;
import frc.robot.components.motors.lib.Motor;
import static frc.robot.subsystems.intake.IntakeConfig.*;

public class IntakeSubsystem extends SubsystemBase {

  final Motor intakeMotor;
  final Motor deployMotor;
  // final PIDController pidController;

  public IntakeSubsystem() {

    var configRight = getIntakeRightConfig();
    var configDeploy = getIntakeDeployConfig();

    // pidController= new PIDController(configDeploy.pidParameters.P, configDeploy.pidParameters.I, configDeploy.pidParameters.D);

    intakeMotor = new Kraken(configRight);
    deployMotor = new Kraken(configDeploy);

    initDashboard();
  }

  @Override
  public void periodic() {}

  public void extend() {

    if (getPosition() >= IntakeConfig.EXTEND_LIMIT) {
      deployMotor.stop();
    }
    else {
      deployMotor.setSpeed(EXTEND_SPEED);
    }
    startIntake();
  }

  public void retract() {

    if (getPosition() <= IntakeConfig.RETRACT_LIMIT) {
      stopIntake();
      deployMotor.stop();
    }
    else {
      deployMotor.setSpeed(RETRACT_SPEED);
      startIntake();
    }
  }

  public double getPosition() {
    return deployMotor.getPosition();
  }

  public void startIntake() {
    intakeMotor.setSpeed(INTAKE_SPEED);
  }

  public void startOuttake() {
    intakeMotor.setSpeed(-INTAKE_SPEED);
  }

  public void stopIntake() {
    intakeMotor.stop();
  }

  public void initDashboard() {
    var tab = Shuffleboard.getTab("Intake");

    tab.addDouble("Deploy Position", () -> getPosition())
      .withPosition(0, 0)
      .withSize(2, 1);

    tab.addDouble("Target Extend", () -> EXTEND_LIMIT)
      .withPosition(0, 1)
      .withSize(2, 1);

    tab.addDouble("Target Retract", () -> RETRACT_LIMIT)
      .withPosition(0, 2)
      .withSize(2, 1);
  }
}
