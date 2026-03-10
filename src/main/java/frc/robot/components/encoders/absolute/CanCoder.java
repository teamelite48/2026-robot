package frc.robot.components.encoders.absolute;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import frc.robot.components.encoders.absolute.lib.AbsoluteEncoder;
import frc.robot.components.encoders.absolute.lib.AbsoluteEncoderConfig;


public class CanCoder implements AbsoluteEncoder {

  AbsoluteEncoderConfig config;
  CANcoder encoder;

  private final StatusSignal<Angle> absPosSignal;

  public CanCoder(AbsoluteEncoderConfig encoderConfig) {
    this.config = encoderConfig;

    if (encoderConfig.canBus == null) {
      encoder = new CANcoder(config.canBusId);
    }
    else {
      encoder = new CANcoder(config.canBusId, config.canBus);
    }

    var canCoderConfig = new CANcoderConfiguration();

    canCoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);  // TODO: need to update logic because we no longer have 0 to 360 output. No clue what this means.
    canCoderConfig.MagnetSensor.withMagnetOffset(-(config.offset / config.positionConversionFactor));
    canCoderConfig.MagnetSensor.withSensorDirection(config.isInverted ? SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive);

    encoder.getConfigurator().apply(canCoderConfig);

    absPosSignal = encoder.getAbsolutePosition();
  }

  public double getPosition() {
    return encoder.getAbsolutePosition().getValueAsDouble() * config.positionConversionFactor;
  }

  public double getRawPosition() {
    // return encoder.getAbsolutePosition().getValueAsDouble();
    return absPosSignal.refresh().getValueAsDouble();
  }

  //  public StatusSignal<Angle> getAbsolutePosition() {
  //    return encoder.getAbsolutePosition();
  // }
}
