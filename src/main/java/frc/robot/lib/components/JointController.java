package frc.robot.lib.components;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class JointController {

  final SparkMax motorController;
  final SparkAbsoluteEncoder absoluteEncoder;
  final SparkClosedLoopController pidController;
  final RelativeEncoder relativeEncoder;

  JointControllerConfig config;
  boolean isInitialized = false;
  double targetAngle;

  public JointController(JointControllerConfig config) {

    this.config = config;
    targetAngle = this.config.initDegrees();

    motorController = new SparkMax(config.motorControllerCanId(), MotorType.kBrushless);
    absoluteEncoder = motorController.getAbsoluteEncoder();
    relativeEncoder = motorController.getEncoder();
    pidController = motorController.getClosedLoopController();

    configureMotorController(config);

    initDashboard(config.shuffleboardTab(), config.jointName());
  }

  public void periodic() {

    if (isInitialized) {
      return;
    } else if (Math.abs(relativeEncoder.getPosition() - config.initDegrees()) < 1.0
        && Math.abs(absoluteEncoder.getPosition() - relativeEncoder.getPosition()) < 0.1) {
      isInitialized = true;
    } else if (config.initDegrees() < 0.0
        && Math.abs(-relativeEncoder.getPosition() - -config.initDegrees()) < 1.0
        && Math.abs(absoluteEncoder.getPosition() - -relativeEncoder.getPosition()) < 0.1) {
      isInitialized = true;
    } else {
        configureMotorController(config);

      if (config.initDegrees() < 0.0) {
        relativeEncoder.setPosition(-absoluteEncoder.getPosition());
      } else {
        relativeEncoder.setPosition(absoluteEncoder.getPosition());
      }
    }
  }

  public void setAngle(double degrees) {

    if (isInitialized == false) {
      return;
    }

    targetAngle = degrees;
    pidController.setReference(targetAngle, SparkMax.ControlType.kPosition);
  }

  public void adjustAngle(double deltaDegrees) {

    if (isInitialized == false) {
      return;
    }

    targetAngle = targetAngle - deltaDegrees;

    pidController.setReference(targetAngle, SparkMax.ControlType.kPosition);
  }

  public double getAngle() {
    return relativeEncoder.getPosition();
  }

  public SparkMax getMotorController() {
    return motorController;
  }

  private void configureMotorController(JointControllerConfig config) {
    var sparkMaxConfig = new SparkMaxConfig()
      .voltageCompensation(12.0)
      .smartCurrentLimit(config.currentLimit())
      .idleMode(config.initIdleMode())
      .inverted(config.isMotorInverted());

    var softLimitConfig = new SoftLimitConfig()
      .forwardSoftLimit(config.forwardLimit())
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimit(config.reverseLimit())
      .reverseSoftLimitEnabled(true);

    sparkMaxConfig.softLimit.apply(softLimitConfig);

    sparkMaxConfig.absoluteEncoder
      .positionConversionFactor(config.absoluteEncoderPositionConversionFactor())
      .zeroOffset((config.absoluteEncoderOffsetDegrees() == 0.0 ? 0.0 : 1.0 - (config.absoluteEncoderOffsetDegrees() / 360.0)))
      .inverted(config.isAbsoluteEncoderInverted());

    sparkMaxConfig.encoder.positionConversionFactor(config.relativeEncoderPositionConversionFactor());

    sparkMaxConfig.closedLoop
      .pid(config.pidParams().P, config.pidParams().I, config.pidParams().D)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    motorController.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void initDashboard(String subsystem, String motorName) {

    var tab = Shuffleboard.getTab(subsystem);

    tab.addDouble(motorName + " Target Angle", () -> targetAngle)
        .withPosition(0, 0);

    tab.addDouble(motorName + " Relative Angle", () -> relativeEncoder.getPosition())
        .withPosition(1, 0);

    tab.addDouble(motorName + " Absolute Angle", () -> absoluteEncoder.getPosition())
        .withPosition(2, 0);

    tab.addBoolean(motorName + " Initialized", () -> isInitialized)
        .withPosition(3, 0);

    tab.addDouble(motorName + " Output Amps", () -> motorController.getOutputCurrent());
  }
}
