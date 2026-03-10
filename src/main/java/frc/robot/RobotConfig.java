package frc.robot;

import com.ctre.phoenix6.CANBus;

import frc.robot.subsystems.led.LedSubsystem.LedMode;

public class RobotConfig {

  public static final CANBus CANIVORE_48 = new CANBus("Canivore48");

  public enum GamePiece {
    Algae(LedMode.Green),
    Coral(LedMode.Purple);

    public final LedMode ledMode;

    private GamePiece(LedMode ledMode) {
      this.ledMode = ledMode;
    }
  }

  public enum VisionTrackingMode {
    Front,
    Rear
  }
}
