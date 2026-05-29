package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Linear-interpolation shot lookup (like WCP CC 2026).
 * Three calibration points: distance → RPM + hood rotation.
 * All calibration values are live-tunable from SmartDashboard/Elastic under "ShotCal/".
 */
public final class ShotMap {

  public static final class ShotSolution {
    private final double distanceMeters;
    private final double hoodRot;
    private final double topRpm;
    private final double midRpm;
    private final double bottomRpm;

    public ShotSolution(
        double distanceMeters,
        double hoodRot,
        double topRpm,
        double midRpm,
        double bottomRpm) {
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

    SmartDashboard.putNumber("ShotCal/Dist1_M", Constants.ShotLookup.kDistanceM[0]);
    SmartDashboard.putNumber("ShotCal/Dist2_M", Constants.ShotLookup.kDistanceM[1]);
    SmartDashboard.putNumber("ShotCal/Dist3_M", Constants.ShotLookup.kDistanceM[2]);
    SmartDashboard.putNumber("ShotCal/Dist4_M", Constants.ShotLookup.kDistanceM[3]);

    SmartDashboard.putNumber("ShotCal/RPM1", Constants.ShotLookup.kTopRpm[0]);
    SmartDashboard.putNumber("ShotCal/RPM2", Constants.ShotLookup.kTopRpm[1]);
    SmartDashboard.putNumber("ShotCal/RPM3", Constants.ShotLookup.kTopRpm[2]);
    SmartDashboard.putNumber("ShotCal/RPM4", Constants.ShotLookup.kTopRpm[3]);

    SmartDashboard.putNumber("ShotCal/Hood1", Constants.ShotLookup.kHoodRot[0]);
    SmartDashboard.putNumber("ShotCal/Hood2", Constants.ShotLookup.kHoodRot[1]);
    SmartDashboard.putNumber("ShotCal/Hood3", Constants.ShotLookup.kHoodRot[2]);
    SmartDashboard.putNumber("ShotCal/Hood4", Constants.ShotLookup.kHoodRot[3]);
  }

  /**
   * Linear-interpolation lookup from live-tunable calibration points.
   * Continuously callable — reads current distance, returns interpolated RPM + hood.
   */
  public static ShotSolution calculate(double distanceMeters) {
    initTunables();

    double[] dists = {
        SmartDashboard.getNumber("ShotCal/Dist1_M", Constants.ShotLookup.kDistanceM[0]),
        SmartDashboard.getNumber("ShotCal/Dist2_M", Constants.ShotLookup.kDistanceM[1]),
        SmartDashboard.getNumber("ShotCal/Dist3_M", Constants.ShotLookup.kDistanceM[2]),
        SmartDashboard.getNumber("ShotCal/Dist4_M", Constants.ShotLookup.kDistanceM[3]),
    };
    double[] rpms = {
        SmartDashboard.getNumber("ShotCal/RPM1", Constants.ShotLookup.kTopRpm[0]),
        SmartDashboard.getNumber("ShotCal/RPM2", Constants.ShotLookup.kTopRpm[1]),
        SmartDashboard.getNumber("ShotCal/RPM3", Constants.ShotLookup.kTopRpm[2]),
        SmartDashboard.getNumber("ShotCal/RPM4", Constants.ShotLookup.kTopRpm[3]),
    };
    double[] hoods = {
        SmartDashboard.getNumber("ShotCal/Hood1", Constants.ShotLookup.kHoodRot[0]),
        SmartDashboard.getNumber("ShotCal/Hood2", Constants.ShotLookup.kHoodRot[1]),
        SmartDashboard.getNumber("ShotCal/Hood3", Constants.ShotLookup.kHoodRot[2]),
        SmartDashboard.getNumber("ShotCal/Hood4", Constants.ShotLookup.kHoodRot[3]),
    };

    double rpm = LinearInterpolation.lookup(dists, rpms, distanceMeters);
    double hoodRot = LinearInterpolation.lookup(dists, hoods, distanceMeters);

    return new ShotSolution(distanceMeters, hoodRot, rpm, rpm, rpm);
  }
}
