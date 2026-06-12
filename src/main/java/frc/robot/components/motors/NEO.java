package frc.robot.components.motors;

//import com.ctre.phoenix6.configs.Slot0Configs;
//import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.components.motors.lib.Motor;
import frc.robot.components.motors.lib.MotorConfig;

public class NEO implements Motor {

  final SparkMax sparkMax;
  final SparkClosedLoopController pidController;
  final RelativeEncoder relativeEncoder;

  final MotorConfig config;

  public NEO(MotorConfig motorConfig) {

    this.config = motorConfig;

    sparkMax = new SparkMax(motorConfig.canBusId, MotorType.kBrushless);
    relativeEncoder = sparkMax.getEncoder();
    pidController = sparkMax.getClosedLoopController();

    var sparkMaxConfig = new SparkMaxConfig();

    sparkMaxConfig.voltageCompensation(12.0);

    if (config.currentLimit != null) {
      sparkMaxConfig.smartCurrentLimit(config.currentLimit);
    }

    if (config.velocityConversionFactor != null) {
      // For Angle Controller in Drive Subsystem
      sparkMaxConfig.encoder.velocityConversionFactor(config.velocityConversionFactor / 60.0);
    }

    sparkMaxConfig.inverted(motorConfig.isInverted);
    sparkMaxConfig.idleMode(motorConfig.isBrakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast);

    sparkMaxConfig.encoder.positionConversionFactor(motorConfig.positionConversionFactor);

    if (config.initialPosition != null) {
      setInitialPosition();
    }

    if (config.pidParameters != null) {
      var pid = config.pidParameters;
      sparkMaxConfig.closedLoop.pid(pid.P, pid.I, pid.D);
      sparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    }

    if (config.forwardLimit != null || config.reverseLimit != null) {

      var softLimitConfig = new SoftLimitConfig();

      if (config.forwardLimit != null) {
        softLimitConfig
          .forwardSoftLimitEnabled(true)
          .forwardSoftLimit(config.forwardLimit);
      }

      if (config.reverseLimit != null) {
        softLimitConfig
          .reverseSoftLimitEnabled(true)
          .reverseSoftLimit(config.reverseLimit);
      }

      if (config.maxForwardSpeed != null || config.maxReverseSpeed != null) {

        double maxRPM = 11000;

        double maxVelocity = maxRPM * Math.min(
          (config.maxForwardSpeed != 0) ? config.maxForwardSpeed : 0.0,
          (config.maxReverseSpeed != 0) ? config.maxReverseSpeed : 0.0
        );

        sparkMaxConfig.closedLoop.maxMotion.cruiseVelocity(maxVelocity);
      }

      sparkMaxConfig.softLimit.apply(softLimitConfig);
    }

    sparkMax.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public double getPosition() {
    return relativeEncoder.getPosition();
  }

  public double getVelocity() {
    return relativeEncoder.getVelocity();
  }

  @Override
  public void setPID(double p, double i, double d, double s, double v) {
      SparkMaxConfig config = new SparkMaxConfig();

      // Use .p, .i, .d, and .ff (for velocity feedforward)
      config.closedLoop
          .p(p)
          .i(i)
          .d(d)
          .velocityFF(v); // 'v' maps to 'ff' in REVLib

      // Static friction (s) is handled in the 'velocity' or 'position' slot 
      // depending on your control mode, but usually via kS:
      config.closedLoop.velocityFF(v); 
      
      // Apply the configuration
      sparkMax.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setPosition(double position) {
    pidController.setReference(position, SparkMax.ControlType.kPosition);
    // sparkMax.setSetpoint(position, SparkMax.ControlType.kPosition);  // 2027 replacement of setReference
  }

  public void setInitialPosition() {
    relativeEncoder.setPosition(config.initialPosition);
  }

  public void setInitialPosition(double position) {
    relativeEncoder.setPosition(position);
  }

  @Override
  public void follow(Motor leader, boolean oppose) {
    // Do Nothing
  }

  public void setSpeed(double speed) {
    sparkMax.set(speed);
  }

  @Override
  public void setVelocity(double rpm) {
    pidController.setSetpoint(rpm, ControlType.kVelocity);
  }

  public void setVoltage(double d) {
    sparkMax.setVoltage(d);
  }

  public void setMotionMagicPosition(double position) {}

  public void setMotionMagicPosition(double position, double feedForwardVolts) {}

  public void stop() {
    sparkMax.stopMotor();
  }
}