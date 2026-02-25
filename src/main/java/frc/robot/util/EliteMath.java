package frc.robot.util;

public class EliteMath {

  public static double clamp(double n, double clampValue) {
    return Math.max(-clampValue, Math.min(clampValue, n));
  }

  public static double clamp(double n, double min, double max) {
    return Math.max(min, Math.min(max, n));
  }

  public static double map(double value, double inMin, double inMax, double outMin, double outMax) {
    if (value < inMin) value = inMin;
    if (value > inMax) value = inMax;

    double inRange = inMax - inMin;
    if (inRange == 0) {
        // Avoid division by zero - just return the outMin or outMax as a fallback
        return (value <= inMin) ? outMin : outMax;
    }

    double outRange = outMax - outMin;
    double scaled = (value - inMin) / inRange;

    return outMin + (scaled * outRange);
  }
}