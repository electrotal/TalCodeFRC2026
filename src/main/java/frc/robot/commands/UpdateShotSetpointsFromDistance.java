package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.FieldTargetUtil;
import frc.robot.util.ShotMap;

/**
 * Updates hood + shooter setpoints from distance-to-hub lookup tables.
 * Distance is measured in meters.
 * Hood targets are in hood rotations.
 * Shooter targets are in RPM.
 */
public class UpdateShotSetpointsFromDistance extends Command {

  private final SwerveSubsystem swerve;
  private final ShooterSubsystem shooter;
  private final HoodSubsystem hood;

  public UpdateShotSetpointsFromDistance(SwerveSubsystem swerve, ShooterSubsystem shooter, HoodSubsystem hood) {
    this.swerve = swerve;
    this.shooter = shooter;
    this.hood = hood;
    addRequirements(shooter, hood);
  }

  @Override
  public void execute() {
    double distanceMeters = FieldTargetUtil.distanceToHubMeters(swerve.getPose());
    ShotMap.ShotSolution shot = ShotMap.calculate(distanceMeters);

    hood.setHoodRot(shot.hoodRot());
    shooter.setTargetRpms(shot.topRpm(), shot.midRpm(), shot.bottomRpm());

    SmartDashboard.putNumber("Shot/DistanceMeters", distanceMeters);
    SmartDashboard.putNumber("Shot/TargetHoodRot", shot.hoodRot());
    SmartDashboard.putNumber("Shot/TargetTopRpm", shot.topRpm());
    SmartDashboard.putNumber("Shot/TargetMidRpm", shot.midRpm());
    SmartDashboard.putNumber("Shot/TargetBottomRpm", shot.bottomRpm());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}