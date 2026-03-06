package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.FieldTargetUtil;
import frc.robot.util.ShootingOnTheMoveUtil;

import java.util.function.DoubleSupplier;

/**
 * While active, you can drive normally (x/y translation),
 * and the robot continuously rotates to face a "virtual goal" that accounts for robot motion.
 */
public class DriveFaceVirtualGoal extends Command {

  private final Command inner;

  public DriveFaceVirtualGoal(SwerveSubsystem swerve, DoubleSupplier xStick, DoubleSupplier yStick) {

    inner =
        new DriveFacePoint(
            swerve,
            xStick,
            yStick,
            () -> {
              // Get robot-relative speeds from swerve, convert to field-relative using current heading.
              ChassisSpeeds robot = swerve.getRobotRelativeSpeeds();
              ChassisSpeeds field = ChassisSpeeds.fromRobotRelativeSpeeds(
                  robot.vxMetersPerSecond,
                  robot.vyMetersPerSecond,
                  robot.omegaRadiansPerSecond,
                  swerve.getHeading()
              );

              Translation2d hub = FieldTargetUtil.hubCenterForAlliance();
              return ShootingOnTheMoveUtil.virtualGoal(hub, swerve.getPose(), field.vxMetersPerSecond, field.vyMetersPerSecond);
            });

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    inner.initialize();
  }

  @Override
  public void execute() {
    inner.execute();
  }

  @Override
  public void end(boolean interrupted) {
    inner.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}