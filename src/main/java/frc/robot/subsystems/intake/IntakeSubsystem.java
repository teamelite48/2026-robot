// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.motors.Kraken;
import frc.robot.components.motors.lib.Motor;
import static frc.robot.subsystems.intake.IntakeConfig.*;

public class IntakeSubsystem extends SubsystemBase {

  final Motor rightMotor;
  final Motor deployMotor;

  public IntakeSubsystem() {

    var configRight = getIntakeRightConfig();
    var configDeploy = getIntakeRightConfig();

    rightMotor = new Kraken(configRight);
    deployMotor = new Kraken(configDeploy);

    initDashboard();
  }

  @Override
  public void periodic() {}

  public void extend() {

    if (getPosition() >= IntakeConfig.EXTEND_LIMIT) {
      // System.out.println("Not Extending");
    }
    else {
      deployMotor.setSpeed(EXTEND_SPEED);
      // System.out.println("Extending");
    }
    startIntake();
  }

public void retract() {

    if (getPosition() <= IntakeConfig.RETRACT_LIMIT) {
      stopIntake();
    }
    else {
      deployMotor.setSpeed(RETRACT_SPEED);
      // System.out.println("Retracting");
    }
  }

  public double getPosition() {
    return deployMotor.getPosition();
  }

  public void startIntake() {
    rightMotor.setSpeed(INTAKE_SPEED);
  }

  public void stopIntake() {
    rightMotor.stop();
  }

  public void initDashboard() {
    var tab = Shuffleboard.getTab("Intake");

    tab.addDouble("Deploy Position", () -> getPosition())
    .withPosition(0, 0)
    .withSize(2, 1);
  }
}
