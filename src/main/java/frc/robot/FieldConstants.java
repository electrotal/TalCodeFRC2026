package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import java.util.Optional;

public final class FieldConstants {

  private FieldConstants() {}

  public static final class Layout {
    public static final AprilTagFieldLayout kFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double kFieldLengthMeters = kFieldLayout.getFieldLength();
    public static final double kFieldWidthMeters = kFieldLayout.getFieldWidth();

    private Layout() {}
  }

  public static Optional<Pose3d> getTagPose3d(int id) {
    return Layout.kFieldLayout.getTagPose(id);
  }

  public static void printAllTagPoses() {
    for (var tag : Layout.kFieldLayout.getTags()) {
      var pose = tag.pose;
      System.out.println(
          "Tag " + tag.ID
              + " x=" + pose.getX()
              + " y=" + pose.getY()
              + " z=" + pose.getZ()
              + " yawDeg=" + pose.getRotation().toRotation2d().getDegrees()
      );
    }
  }

  public static final class Hub {
    public static final double kHubWidthMeters = Units.inchesToMeters(47.0);
    public static final int kNearFaceTagId = 26;

    public static Translation2d getCenter() {
      Optional<Pose3d> nearFaceTagPose = Layout.kFieldLayout.getTagPose(kNearFaceTagId);
      if (nearFaceTagPose.isEmpty()) {
        return new Translation2d(Layout.kFieldLengthMeters / 2.0, Layout.kFieldWidthMeters / 2.0);
      }

      double hubCenterX = nearFaceTagPose.get().getX() + (kHubWidthMeters / 2.0);
      double hubCenterY = Layout.kFieldWidthMeters / 2.0;
      return new Translation2d(hubCenterX, hubCenterY);
    }

    private Hub() {}
  }
}