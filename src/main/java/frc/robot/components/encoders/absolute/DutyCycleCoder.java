package frc.robot.components.encoders.absolute;

import frc.robot.components.encoders.absolute.lib.AbsoluteEncoder;
import frc.robot.components.encoders.absolute.lib.AbsoluteEncoderConfig;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class DutyCycleCoder implements AbsoluteEncoder {

  AbsoluteEncoderConfig config;
  DutyCycleEncoder digitalEncoder;

  public DutyCycleCoder(AbsoluteEncoderConfig encoderConfig) {
    this.config = encoderConfig;

    digitalEncoder = new DutyCycleEncoder(config.id);
    digitalEncoder.setInverted(config.isInverted);

  }

  public double getPosition() {
    return (digitalEncoder.get() + (config.offset / config.positionConversionFactor)) * config.positionConversionFactor;
  }
}