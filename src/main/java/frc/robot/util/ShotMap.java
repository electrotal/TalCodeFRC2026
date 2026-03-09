package frc.robot.util;

import frc.robot.Constants;

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

    public double distanceMeters() {
      return distanceMeters;
    }

    public double hoodRot() {
      return hoodRot;
    }

    public double topRpm() {
      return topRpm;
    }

    public double midRpm() {
      return midRpm;
    }

    public double bottomRpm() {
      return bottomRpm;
    }
  }

  private ShotMap() {}

  public static ShotSolution calculate(double distanceMeters) {
    double hoodRot = LinearInterpolation.lookup(
        Constants.ShotLookup.kDistanceM,
        Constants.ShotLookup.kHoodRot,
        distanceMeters);

    double topRpm = LinearInterpolation.lookup(
        Constants.ShotLookup.kDistanceM,
        Constants.ShotLookup.kTopRpm,
        distanceMeters);

    double midRpm = LinearInterpolation.lookup(
        Constants.ShotLookup.kDistanceM,
        Constants.ShotLookup.kMidRpm,
        distanceMeters);

    double bottomRpm = LinearInterpolation.lookup(
        Constants.ShotLookup.kDistanceM,
        Constants.ShotLookup.kBottomRpm,
        distanceMeters);

    return new ShotSolution(distanceMeters, hoodRot, topRpm, midRpm, bottomRpm);
  }
}