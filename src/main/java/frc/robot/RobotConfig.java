package frc.robot;

import frc.robot.subsystems.led.LedSubsystem.LedMode;

public class RobotConfig {

  public static final String CANIVORE_48 = "Canivore48";

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
