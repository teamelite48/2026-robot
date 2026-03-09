package frc.robot.components.encoders.absolute.lib;

import com.ctre.phoenix6.CANBus;

public class AbsoluteEncoderConfig {
  public CANBus canBus = null;  // Used for canivore
  public final int canBusId;
  public double offset;
  public boolean isInverted = false;
  public double positionConversionFactor;

  public AbsoluteEncoderConfig (int canBusId) {
    this.canBusId = canBusId;
  }

  public AbsoluteEncoderConfig (int canBusId, CANBus bus) {
    this.canBus = bus;
    this.canBusId = canBusId;
  }
}
