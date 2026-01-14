package frc.robot.subsystems.shoulder;

import frc.robot.components.encoders.absolute.lib.AbsoluteEncoderConfig;
import frc.robot.components.motors.lib.MotorConfig;
import frc.robot.lib.PIDParameters;

public class ShoulderConfig {

  public static final double MANUAL_TARGET_MODIFIER = 0.45;
  public static final double MAX_SPEED = 0.5;
  public static final double ANGLE_GAIN = 1.0; //1.5;
  public static final double LENGTH_GAIN = 1.0; //1.55; 5.0
  public static final PIDParameters PID_PARAMETERS = new PIDParameters(0.05, 0.0, 0.0);
  public static final double TARGET_THRESHOLD = 1.0;
  public static final double INITIAL_POSITION = 90.0;


  public static MotorConfig getMotorConfig(double initialPosition) {
    var config = new MotorConfig(12);

    config.isInverted = true;
    config.positionConversionFactor = (1.0/133.333) * 360.0;
    config.isBrakeModeEnabled = true;
    config.initialPosition = initialPosition;
    config.isBrakeModeEnabled = true;

    return config;
  }

  public static AbsoluteEncoderConfig getAbsoluteEncoderConfig() {
    var config = new AbsoluteEncoderConfig(0);

    config.offset = 38.3;
    config.isInverted = true;
    config.positionConversionFactor = 360.0;

    return config;
  }
}