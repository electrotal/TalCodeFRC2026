package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.SwerveSubsystem;

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

  /** Virtual (lead) goal for the current robot motion — shared by the aim command and the
   *  shooter/hood distance follow so both target the same point. */
  public static Translation2d virtualGoalFromSwerve(SwerveSubsystem swerve, Translation2d hubCenter) {
    ChassisSpeeds robot = swerve.getRobotRelativeSpeeds();
    ChassisSpeeds field = ChassisSpeeds.fromRobotRelativeSpeeds(
        robot.vxMetersPerSecond, robot.vyMetersPerSecond, robot.omegaRadiansPerSecond, swerve.getHeading());
    return virtualGoal(hubCenter, swerve.getPose(), field.vxMetersPerSecond, field.vyMetersPerSecond);
  }
}