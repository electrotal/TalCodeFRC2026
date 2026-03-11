package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Periodically injects Limelight pose measurements into the swerve pose estimator.
 */
public class VisionFusionSubsystem extends SubsystemBase {

  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;

  // True once odometry has been seeded from a valid vision pose.
  // Before seeding, odometry starts at (0,0) and the jump filter would reject
  // any real correction, so we hard-reset on the first good measurement instead.
  private boolean odometrySeeded = false;

  public VisionFusionSubsystem(SwerveSubsystem swerve, VisionSubsystem vision) {
    this.swerve = swerve;
    this.vision = vision;
    SmartDashboard.putBoolean("Vision/Fusion/Seeded", false);
  }

  @Override
  public void periodic() {
    if (!swerve.isVisionFusionSupported()) return;

    var maybeEstimate = vision.getBotPoseWpiBlue();

    // Diagnostics — visible in Elastic/SmartDashboard under Vision/Fusion/
    SmartDashboard.putBoolean("Vision/Fusion/HasEstimate", maybeEstimate.isPresent());
    SmartDashboard.putBoolean("Vision/Fusion/HasTarget", vision.hasTarget());

    if (maybeEstimate.isEmpty()) {
      SmartDashboard.putString("Vision/Fusion/RejectReason", "no pose");
      return;
    }

    var meas = maybeEstimate.get();
    SmartDashboard.putNumber("Vision/Fusion/TagCount", meas.tagCount);
    SmartDashboard.putNumber("Vision/Fusion/LatencySec", meas.latencySeconds);
    SmartDashboard.putNumber("Vision/Fusion/PoseX", meas.pose.getX());
    SmartDashboard.putNumber("Vision/Fusion/PoseY", meas.pose.getY());

    if (!vision.hasTarget()) {
      SmartDashboard.putString("Vision/Fusion/RejectReason", "no target (tv=0)");
      return;
    }
    if (meas.tagCount < Constants.VisionConstants.kMinTagCount) {
      SmartDashboard.putString("Vision/Fusion/RejectReason", "tagCount=" + meas.tagCount + " < min=" + Constants.VisionConstants.kMinTagCount);
      return;
    }
    if (meas.latencySeconds > Constants.VisionConstants.kMaxLatencySeconds) {
      SmartDashboard.putString("Vision/Fusion/RejectReason", "latency=" + meas.latencySeconds + "s too high");
      return;
    }

    // On first valid measurement, hard-reset odometry so the jump filter works correctly
    // going forward. Without this, the robot starts at (0,0) and every real tag reading
    // gets rejected because it looks like a giant jump.
    if (!odometrySeeded) {
      swerve.resetPose(meas.pose);
      odometrySeeded = true;
      SmartDashboard.putString("Vision/Fusion/RejectReason", "SEEDED");
      SmartDashboard.putBoolean("Vision/Fusion/Seeded", true);
      return;
    }

    Pose2d current = swerve.getPose();
    double jump = current.getTranslation().getDistance(meas.pose.getTranslation());
    SmartDashboard.putNumber("Vision/Fusion/PoseJumpM", jump);

    if (jump > Constants.VisionConstants.kMaxPoseJumpMeters) {
      SmartDashboard.putString("Vision/Fusion/RejectReason", "jump=" + String.format("%.2f", jump) + "m too large");
      return;
    }

    swerve.addVisionMeasurement(meas.pose, meas.timestampSeconds);
    SmartDashboard.putString("Vision/Fusion/RejectReason", "ACCEPTED");
  }
}