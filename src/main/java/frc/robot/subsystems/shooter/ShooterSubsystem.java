// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.shooter;

// import edu.wpi.first.wpilibj.RobotState;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotContainer;
// import frc.robot.components.motors.Kraken;
// import frc.robot.components.motors.lib.Motor;
// import frc.robot.components.motors.lib.MotorConfig;
// import frc.robot.lib.LinearInterpolator;
// import frc.robot.subsystems.led.LedSubsystem.LedMode;

// import static frc.robot.subsystems.shooter.ShooterConfig.*;

// import java.util.function.Supplier;

// public class ShooterSubsystem extends SubsystemBase {

//   final Motor leftMotor;
//   final Motor rightMotor;
//   final MotorConfig leftMotorConfig;

//   boolean isShooterOn = false;
//   boolean isOnSpeed = false;
//   double lastOnSpeedRpm = 0.0;
//   double targetRPM = 0.0;

//   boolean isRangeBasedRPMOn = false;

//   LinearInterpolator feetToRpmInterpolator = new LinearInterpolator(FEET_TO_RPM_MAP);
//   Supplier<Double> feetFromTargetSupplier;

//   // double calculatedRPM;

//   public ShooterSubsystem(Supplier<Double> feetFromTargetSupplier) {

//     this.feetFromTargetSupplier = feetFromTargetSupplier;

//     this.leftMotorConfig = getShooterLeftConfig();
//     var configRight = getShooterRightConfig();

//     leftMotor = new Kraken(leftMotorConfig);  // Leader motor
//     rightMotor = new Kraken(configRight);

//     rightMotor.follow(leftMotor, true);

//     // leftMotor.setSpeed(IDLE_MOTOR_POWER);

//     initDashboard();
//   }

//   @Override
//   public void periodic() {

//     if (isShooterOn == false) {
//       leftMotor.stop();
//     }
//     else if (RobotContainer.isAimAssistEnabled){
//       targetRPM = feetToRpmInterpolator.calculate(feetFromTargetSupplier.get());
//       setShooterRPM(targetRPM);
//     }
//   }


//     // var currentOnSpeedRpm = RobotContainer.isAimAssistEnabled
//     //   ? feetToRpmInterpolator.calculate(feetFromTargetSupplier.get())
//     //   : ON_SPEED_RPM;

//     // if (Math.abs(currentOnSpeedRpm - lastOnSpeedRpm) > 5.0) {
//     //   isOnSpeed = false;
//     // }

//     // if (isOnSpeed == false && getShooterRPM() > currentOnSpeedRpm) {
//     //   isOnSpeed = true;
//     //   RobotContainer.ledSubsystem.setLedMode(LedMode.Green);
//     // }

//     // else if (getShooterRPM() < IDLE_RPM && RobotState.isTeleop()){
//     //   RobotContainer.ledSubsystem.setLedMode(LedMode.Red);
//     // }

//     // lastOnSpeedRpm = currentOnSpeedRpm;
//   // }

//   // private double convertMotorPowerToRPM() {

//   //   double motorRPS = 60.0;
//   //   double motorShaftRPM = leftMotor.getVelocity() * motorRPS;
//   //   double wheelRPM = motorShaftRPM / leftMotorConfig.positionConversionFactor;

//   //   return wheelRPM;
//   // }

//   // private double convertRPMToMotorPower(double rpm) {
//   //   double motorRPS = 60.0;
//   //   double wheelRPM = rpm * leftMotorConfig.positionConversionFactor;
//   //   double power = (wheelRPM / (leftMotor.getVelocity() * motorRPS));

//   //   return power;
//   // }

//   // private double updateCalculatedRPM(double rpm) {
//   //   calculatedRPM = rpm;
//   //   return calculatedRPM;
//   // }

//   // public double getCalculatedRPM() {
//   //   return calculatedRPM;
//   // }

//   public boolean isShooterOn() {
//       return isShooterOn;
//   }

//   public void setSpeed(double speed) {
//     isOnSpeed = false;
//     isShooterOn = true;
//     leftMotor.setSpeed(speed);
//   }

//   public void idleShooter() {
//     targetRPM = IDLE_RPM;
//   }

//   public void stop() {
//     isShooterOn = false;
//     isOnSpeed = false;
//     leftMotor.stop();
//   }

//   public boolean getIsOnSpeed() {
//     return isOnSpeed;
//   }

//   private double getShooterRPM() {
//     return leftMotor.getVelocity();
//   }

//   public void setShooterRPM(double targetRpm) {
//     leftMotor.setVelocity(targetRpm);
//   }

//   public double getTargetRPM() {
//     return targetRPM;
//   }

//   public void bumpUpRPM() {
//     targetRPM += RPM_BUMP;
//   }

//   public void bumpDownRPM() {
//     targetRPM -= RPM_BUMP;
//   }

//   public void initDashboard() {
//     var tab = Shuffleboard.getTab("Shooter");

//     tab.addBoolean("Shooter Enabled", () -> isShooterOn)
//       .withPosition(0, 0)
//       .withSize(1, 1);

//     tab.addDouble("Current RPM Shooter", () -> getShooterRPM())
//       .withPosition(1, 0)
//       .withSize(2, 1);

//     tab.addBoolean("On Speed", () -> getIsOnSpeed())
//       .withPosition(0, 1)
//       .withSize(1, 1);
//   }
// }

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConfig.*;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.components.motors.Kraken;
import frc.robot.components.motors.lib.Motor;
import frc.robot.components.motors.lib.MotorConfig;
import frc.robot.lib.LinearInterpolator;

public class ShooterSubsystem extends SubsystemBase {

  public enum ShooterMode {
    OFF,
    IDLE,
    MANUAL,
    AUTO_RANGE
  }

  private final Motor leftMotor;
  private final Motor rightMotor;
  private final MotorConfig leftMotorConfig;

  private final Supplier<Double> feetFromTargetSupplier;
  private final LinearInterpolator feetToRpmInterpolator = new LinearInterpolator(FEET_TO_RPM_MAP);

  private ShooterMode shooterMode = ShooterMode.OFF;
  private ShooterConfig.ShooterPreset manualPreset = ShooterConfig.ShooterPreset.MEDIUM;

  private double manualTargetRPM = MEDIUM_RPM;
  private double targetRPM = IDLE_RPM;
  private boolean isOnSpeed = false;

  public ShooterSubsystem(Supplier<Double> feetFromTargetSupplier) {
    this.feetFromTargetSupplier = feetFromTargetSupplier;

    this.leftMotorConfig = getShooterLeftConfig();
    var rightConfig = getShooterRightConfig();

    leftMotor = new Kraken(leftMotorConfig);
    rightMotor = new Kraken(rightConfig);

    rightMotor.follow(leftMotor, true);

    initDashboard();
  }

  @Override
  public void periodic() {
    switch (shooterMode) {
      case OFF:
        targetRPM = 0.0;
        leftMotor.stop();
        break;

      case IDLE:
        targetRPM = IDLE_RPM;
        leftMotor.setVelocity(targetRPM);
        break;

      case MANUAL:
        targetRPM = manualTargetRPM;
        leftMotor.setVelocity(targetRPM);
        break;

      case AUTO_RANGE:
        double feet = feetFromTargetSupplier.get();
        targetRPM = feetToRpmInterpolator.calculate(feet);
        leftMotor.setVelocity(targetRPM);
        break;
    }

    isOnSpeed = Math.abs(getShooterRPM() - targetRPM) <= ON_SPEED_TOLERANCE_RPM;
  }

  public void setOff() {
    shooterMode = ShooterMode.OFF;
  }

  public void setIdle() {
    shooterMode = ShooterMode.IDLE;
  }

  public void setManualPreset(ShooterConfig.ShooterPreset preset) {
    manualPreset = preset;

    switch (preset) {
      case LOW:
        manualTargetRPM = LOW_RPM;
        break;
      case MEDIUM:
        manualTargetRPM = MEDIUM_RPM;
        break;
      case HIGH:
        manualTargetRPM = HIGH_RPM;
        break;
      case PASS:
        manualTargetRPM = PASS_RPM;
        break;
    }

    shooterMode = ShooterMode.MANUAL;
  }

  public void setLowRPM() {
    setManualPreset(ShooterPreset.LOW);
  }

  public void setMediumRPM() {
    setManualPreset(ShooterPreset.MEDIUM);
  }

  public void setHighRPM() {
    setManualPreset(ShooterPreset.HIGH);
  }

  public void setManualRPM(double rpm) {
    manualTargetRPM = rpm;
    shooterMode = ShooterMode.MANUAL;
  }

  public void setAutoRangeEnabled(boolean enabled) {
    shooterMode = enabled ? ShooterMode.AUTO_RANGE : ShooterMode.MANUAL;
  }

  public void useAimAssistOrPreset(ShooterConfig.ShooterPreset preset) {
    if (RobotContainer.isAimAssistEnabled) {
      shooterMode = ShooterMode.AUTO_RANGE;
    } else {
      setManualPreset(preset);
    }
  }

  public void bumpUpRPM() {
    if (shooterMode != ShooterMode.MANUAL) {
      shooterMode = ShooterMode.MANUAL;
      manualTargetRPM = targetRPM;
    }
    manualTargetRPM += RPM_BUMP;
  }

  public void bumpDownRPM() {
    if (shooterMode != ShooterMode.MANUAL) {
      shooterMode = ShooterMode.MANUAL;
      manualTargetRPM = targetRPM;
    }
    manualTargetRPM -= RPM_BUMP;
  }

  public boolean isShooterOn() {
    return shooterMode != ShooterMode.OFF && shooterMode != ShooterMode.IDLE;
  }

  public boolean isIdling() {
    return shooterMode == ShooterMode.IDLE;
  }

  public boolean getIsOnSpeed() {
    return isOnSpeed;
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public ShooterMode getShooterMode() {
    return shooterMode;
  }

  public ShooterConfig.ShooterPreset getManualPreset() {
    return manualPreset;
  }

  private double getShooterRPM() {
    return leftMotor.getVelocity();
  }

  public void initDashboard() {
    var tab = Shuffleboard.getTab("Shooter");

    tab.addString("Shooter Mode", () -> shooterMode.name())
      .withPosition(0, 0)
      .withSize(2, 1);

    tab.addBoolean("Shooter Enabled", this::isShooterOn)
      .withPosition(0, 1)
      .withSize(1, 1);

    tab.addDouble("Target RPM", this::getTargetRPM)
      .withPosition(1, 1)
      .withSize(2, 1);

    tab.addDouble("Current RPM", this::getShooterRPM)
      .withPosition(1, 2)
      .withSize(2, 1);

    tab.addBoolean("On Speed", this::getIsOnSpeed)
      .withPosition(0, 2)
      .withSize(1, 1);
  }
}
