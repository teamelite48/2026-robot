// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.led.LedConfig.*;

public class LedSubsystem extends SubsystemBase {

  PWMSparkMax motorController = new PWMSparkMax(BLINKIN_PWM_PORT);

  LedMode defaultMode;
  LedMode currentMode;

  public enum LedMode {
    Confetti(-0.87),
    Green(0.77),
    OceanRainbow(-0.95),
    Purple(0.91),
    Red(0.61),
    Yellow(0.69),
    GoldHeartBeat(-0.07);

    double value;

    private LedMode(double value) {
      this.value = value;
    }
  }

  public LedSubsystem() {
    defaultMode = LedMode.Confetti;
    setLedMode(defaultMode);
    initDashBoard();
  }

  public void enabledDefaultMode() {
    setLedMode(defaultMode);
  }

  public void setLedMode(LedMode mode) {
    currentMode = mode;
    this.motorController.set(currentMode.value);
  }

  private void initDashBoard() {
    var tab = Shuffleboard.getTab("LED");
    tab.addString("State", () -> currentMode.toString());
  }
}
