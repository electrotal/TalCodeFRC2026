package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Linear-interpolation shot lookup. Distance (m, robot -> hub center) -> hood rotation + RPM.
 * Calibration points come from Constants.ShotLookup and are live-tunable from Elastic under
 * "ShotCal/" (DistN_M, HoodN, RPMN). Works with any number of points (3+).
 */
public final class ShotMap {

  public static final class ShotSolution {
    private final double distanceMeters, hoodRot, topRpm, midRpm, bottomRpm;

    public ShotSolution(double distanceMeters, double hoodRot, double topRpm, double midRpm, double bottomRpm) {
      this.distanceMeters = distanceMeters;
      this.hoodRot = hoodRot;
      this.topRpm = topRpm;
      this.midRpm = midRpm;
      this.bottomRpm = bottomRpm;
    }

    public double distanceMeters() { return distanceMeters; }
    public double hoodRot() { return hoodRot; }
    public double topRpm() { return topRpm; }
    public double midRpm() { return midRpm; }
    public double bottomRpm() { return bottomRpm; }
  }

  private static boolean initialized = false;

  private ShotMap() {}

  private static void initTunables() {
    if (initialized) return;
    initialized = true;
    for (int i = 0; i < Constants.ShotLookup.kDistanceM.length; i++) {
      int n = i + 1;
      SmartDashboard.putNumber("ShotCal/Dist" + n + "_M", Constants.ShotLookup.kDistanceM[i]);
      SmartDashboard.putNumber("ShotCal/RPM" + n, Constants.ShotLookup.kTopRpm[i]);
      SmartDashboard.putNumber("ShotCal/Hood" + n, Constants.ShotLookup.kHoodRot[i]);
    }
  }

  /** Interpolated shot for a distance (m). Reads live ShotCal/* values, defaulting to constants. */
  public static ShotSolution calculate(double distanceMeters) {
    initTunables();

    int len = Constants.ShotLookup.kDistanceM.length;
    double[] dists = new double[len];
    double[] rpms = new double[len];
    double[] hoods = new double[len];
    for (int i = 0; i < len; i++) {
      int n = i + 1;
      dists[i] = SmartDashboard.getNumber("ShotCal/Dist" + n + "_M", Constants.ShotLookup.kDistanceM[i]);
      rpms[i] = SmartDashboard.getNumber("ShotCal/RPM" + n, Constants.ShotLookup.kTopRpm[i]);
      hoods[i] = SmartDashboard.getNumber("ShotCal/Hood" + n, Constants.ShotLookup.kHoodRot[i]);
    }

    double rpm = LinearInterpolation.lookup(dists, rpms, distanceMeters);
    double hoodRot = LinearInterpolation.lookup(dists, hoods, distanceMeters);
    return new ShotSolution(distanceMeters, hoodRot, rpm, rpm, rpm);
  }
}
