package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Periodically injects Limelight pose measurements into the swerve pose estimator.
 */
public class VisionFusionSubsystem extends SubsystemBase {

  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;

  public VisionFusionSubsystem(SwerveSubsystem swerve, VisionSubsystem vision) {
    this.swerve = swerve;
    this.vision = vision;
  }

  @Override
  public void periodic() {
    if (!swerve.isVisionFusionSupported()) return;

    vision.getBotPoseWpiBlue().ifPresent(meas -> {
      if (!vision.hasTarget()) return;
      if (meas.tagCount < Constants.VisionConstants.kMinTagCount) return;
      if (meas.latencySeconds > Constants.VisionConstants.kMaxLatencySeconds) return;

      // Reject huge jumps.
      Pose2d current = swerve.getPose();
      double jump = current.getTranslation().getDistance(meas.pose.getTranslation());
      if (jump > Constants.VisionConstants.kMaxPoseJumpMeters) return;

      swerve.addVisionMeasurement(meas.pose, meas.timestampSeconds);
    });
  }
}