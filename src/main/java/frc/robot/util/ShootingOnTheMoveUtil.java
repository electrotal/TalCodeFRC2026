package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class ShootingOnTheMoveUtil {

  private ShootingOnTheMoveUtil() {}

  // ⚙️ Tune this on the robot with real shots
  // It represents total time from measurement to ball exit, including vision latency, spinup, and flight.
  public static double kLeadTimeSeconds = 0.25;

  public static Translation2d virtualGoal(
      Translation2d hubCenter,
      Pose2d robotPose,
      double vxFieldMetersPerSec,
      double vyFieldMetersPerSec) {

    // Simple lead: aim ahead of hub by subtracting robot motion over lead time
    // This makes the robot point slightly "upstream" relative to its motion.
    return new Translation2d(
        hubCenter.getX() - vxFieldMetersPerSec * kLeadTimeSeconds,
        hubCenter.getY() - vyFieldMetersPerSec * kLeadTimeSeconds
    );
  }
}