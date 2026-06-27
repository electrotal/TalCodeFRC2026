package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.FieldTargetUtil;
import frc.robot.util.ShootingOnTheMoveUtil;

import java.util.function.DoubleSupplier;

/**
 * While active, you can drive normally (x/y translation), and the robot continuously rotates to
 * face a "virtual goal" that leads the hub by the robot's motion (shooting on the move). The
 * shooter RPM + hood angle should be driven from the SAME virtual goal — see the parallel
 * UpdateShotSetpointsFromDistance wired in RobotContainer.
 */
public class DriveFaceVirtualGoal extends Command {

  private final Command inner;

  public DriveFaceVirtualGoal(
      SwerveSubsystem swerve,
      DoubleSupplier xStick,
      DoubleSupplier yStick,
      DoubleSupplier rotationOverrideRadPerSec) {

    inner =
        new DriveFacePoint(
            swerve,
            xStick,
            yStick,
            () -> ShootingOnTheMoveUtil.virtualGoalFromSwerve(swerve, FieldTargetUtil.hubCenterForAlliance()),
            rotationOverrideRadPerSec);

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
