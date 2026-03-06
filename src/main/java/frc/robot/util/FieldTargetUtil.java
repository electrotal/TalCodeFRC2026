package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;

/**
 * Helpers for field targets and alliance-aware mirroring.
 */
public final class FieldTargetUtil {

  private FieldTargetUtil() {}

  public static Translation2d hubCenterForAlliance() {
    Translation2d blueHub = FieldConstants.Hub.getCenter();
    double fieldLength = FieldConstants.Layout.kFieldLengthMeters;

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      // Mirror X to the red side.
      return new Translation2d(fieldLength - blueHub.getX(), blueHub.getY());
    }
    return blueHub;
  }

  public static double distanceToHubMeters(Pose2d robotPose) {
    return robotPose.getTranslation().getDistance(hubCenterForAlliance());
  }
}