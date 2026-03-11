package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

  // Simple container for a vision pose measurement with timestamp
  public static final class VisionMeasurement {
    public final Pose2d pose;
    public final double timestampSeconds;
    public final int tagCount;
    public final double latencySeconds;

    public VisionMeasurement(Pose2d pose, double timestampSeconds, int tagCount, double latencySeconds) {
      this.pose = pose;
      this.timestampSeconds = timestampSeconds;
      this.tagCount = tagCount;
      this.latencySeconds = latencySeconds;
    }
  }

  private final NetworkTable limelightTable;

  public VisionSubsystem() {
    this("limelight");
  }

  public VisionSubsystem(String tableName) {
    limelightTable = NetworkTableInstance.getDefault().getTable(tableName);

    // Publish Limelight stream URL directly to the CameraPublisher NT table
    // so Elastic discovers it as a Camera Stream widget source
    NetworkTableInstance.getDefault()
        .getTable("CameraPublisher")
        .getSubTable("Limelight")
        .getEntry("streams")
        .setStringArray(new String[]{"mjpeg:http://limelight.local:5800/stream.mjpeg"});
  }

  // tv == 1 means Limelight has a valid target
  public boolean hasTarget() {
    return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
  }

  /** True if we currently have a parseable pose array from Limelight. */
  public boolean hasValidEstimate() {
    return getBotPoseWpiBlue().isPresent();
  }

  /** Best-effort latency (seconds). Returns 0.0 when no estimate is available. */
  public double getTotalLatencySeconds() {
    return getBotPoseWpiBlue().map(m -> m.latencySeconds).orElse(0.0);
  }

  // Reads botpose in WPILib blue field coordinate system, timestamps it using latency
  public Optional<VisionMeasurement> getBotPoseWpiBlue() {
    double[] arr = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
    if (arr.length < 6) return Optional.empty();

    double xMeters = arr[0];
    double yMeters = arr[1];
    double yawDeg = arr[5];

    // Limelight publishes all-zeros when no valid estimate — reject it
    if (xMeters == 0.0 && yMeters == 0.0) return Optional.empty();

    // Prefer latency from the pose array, else use cl + tl
    double totalLatencyMs;
    int tagCount;

    if (arr.length >= 7) {
      totalLatencyMs = arr[6];
    } else {
      double cl = limelightTable.getEntry("cl").getDouble(0.0);
      double tl = limelightTable.getEntry("tl").getDouble(0.0);
      totalLatencyMs = cl + tl;
    }

    if (arr.length >= 8) {
      tagCount = (int) Math.round(arr[7]);
    } else {
      tagCount = hasTarget() ? 1 : 0;
    }

    double latencySeconds = totalLatencyMs / 1000.0;
    double timestamp = Timer.getFPGATimestamp() - latencySeconds;

    Pose2d pose = new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(yawDeg));
    return Optional.of(new VisionMeasurement(pose, timestamp, tagCount, latencySeconds));
  }
}