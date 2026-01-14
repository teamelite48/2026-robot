package frc.robot.components.encoders.absolute.lib;

public class AbsoluteEncoderConfig {
  public final int id;
  public double offset;
  public boolean isInverted = false;
  public double positionConversionFactor;

  public AbsoluteEncoderConfig (int id) {
    this.id = id;
  }
}
