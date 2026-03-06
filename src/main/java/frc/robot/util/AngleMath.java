package frc.robot.util;

public final class AngleMath {

  private AngleMath() {}

  public static double wrapDeg(double deg) {
    double x = deg % 360.0;
    if (x >= 180.0) x -= 360.0;
    if (x < -180.0) x += 360.0;
    return x;
  }

  public static double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
  }
}