package frc.robot.components.encoders.absolute;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.components.encoders.absolute.lib.AbsoluteEncoder;
import frc.robot.components.encoders.absolute.lib.AbsoluteEncoderConfig;


public class CanCoder implements AbsoluteEncoder {

  AbsoluteEncoderConfig config;
  CANcoder encoder;

  public CanCoder(AbsoluteEncoderConfig encoderConfig) {
    this.config = encoderConfig;

    encoder = new CANcoder(config.id);

    var canCoderConfig = new CANcoderConfiguration();

    canCoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
    canCoderConfig.MagnetSensor.withMagnetOffset(-(config.offset / config.positionConversionFactor));
    canCoderConfig.MagnetSensor.withSensorDirection(config.isInverted ? SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive);

    encoder.getConfigurator().apply(canCoderConfig);
  }

  public double getPosition() {
    return encoder.getAbsolutePosition().getValueAsDouble() * config.positionConversionFactor;
  }
}
