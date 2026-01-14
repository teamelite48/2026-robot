package frc.robot.util;

public class EliteMath {

  public static double clamp(double n, double clampValue) {
    return Math.max(-clampValue, Math.min(clampValue, n));
  }

  public static double clamp(double n, double min, double max) {
    return Math.max(min, Math.min(max, n));
  }
}