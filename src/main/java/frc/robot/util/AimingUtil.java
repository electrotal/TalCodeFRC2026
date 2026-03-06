package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class AimingUtil {

  private AimingUtil() {}

  public static Rotation2d headingToPoint(Pose2d robotPose, Translation2d targetPoint) {
    double dx = targetPoint.getX() - robotPose.getX();
    double dy = targetPoint.getY() - robotPose.getY();
    return new Rotation2d(Math.atan2(dy, dx));
  }

  public static double distanceToPointMeters(Pose2d robotPose, Translation2d targetPoint) {
    double dx = targetPoint.getX() - robotPose.getX();
    double dy = targetPoint.getY() - robotPose.getY();
    return Math.hypot(dx, dy);
  }
}