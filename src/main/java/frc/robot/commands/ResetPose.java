package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetPose extends InstantCommand {

  // Resets odometry to a known pose, call at start of autonomous
  public ResetPose(SwerveSubsystem swerve, Pose2d pose) {
    super(() -> swerve.resetPose(pose), swerve);
  }
}