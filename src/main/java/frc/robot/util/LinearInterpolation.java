package frc.robot.util;

public final class LinearInterpolation {

  private LinearInterpolation() {}

  // Linear interpolate between two points
  public static double lerp(double x0, double y0, double x1, double y1, double x) {
    if (x1 == x0) return y0;
    double t = (x - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
  }

  // Lookup y(x) from a piecewise linear table (xs must be sorted)
  public static double lookup(double[] xs, double[] ys, double x) {
    if (xs.length == 0 || ys.length == 0 || xs.length != ys.length) return 0.0;
    if (xs.length == 1) return ys[0];

    if (x <= xs[0]) return ys[0];
    int last = xs.length - 1;
    if (x >= xs[last]) return ys[last];

    for (int i = 0; i < last; i++) {
      if (x >= xs[i] && x <= xs[i + 1]) {
        return lerp(xs[i], ys[i], xs[i + 1], ys[i + 1], x);
      }
    }
    return ys[last];
  }
}