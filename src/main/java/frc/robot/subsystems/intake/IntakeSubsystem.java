// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.motors.Kraken;
import frc.robot.components.motors.lib.Motor;
import static frc.robot.subsystems.intake.IntakeConfig.*;

public class IntakeSubsystem extends SubsystemBase {

  final Motor intakeMotor;

  public IntakeSubsystem() {

    var configRight = getIntakeRightConfig();

    intakeMotor = new Kraken(configRight);
  }

  @Override
  public void periodic() {}

  public void startIntake() {
    intakeMotor.setSpeed(INTAKE_SPEED);
  }

  public void startOuttake() {
    intakeMotor.setSpeed(-INTAKE_SPEED);
  }

  public void stop() {
    intakeMotor.stop();
  }
}
