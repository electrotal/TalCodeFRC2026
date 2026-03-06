package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.FieldTargetUtil;
import frc.robot.util.LinearInterpolation;

/**
 * Updates hood + shooter setpoints from distance-to-hub lookup tables.
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
    double d = FieldTargetUtil.distanceToHubMeters(swerve.getPose());

    double hoodRot = LinearInterpolation.lookup(Constants.ShotLookup.kDistanceM, Constants.ShotLookup.kHoodRot, d);
    double top = LinearInterpolation.lookup(Constants.ShotLookup.kDistanceM, Constants.ShotLookup.kTopRpm, d);
    double mid = LinearInterpolation.lookup(Constants.ShotLookup.kDistanceM, Constants.ShotLookup.kMidRpm, d);
    double bot = LinearInterpolation.lookup(Constants.ShotLookup.kDistanceM, Constants.ShotLookup.kBottomRpm, d);

    hood.setHoodRot(hoodRot);
    shooter.setTargetRpm(ShooterSubsystem.ShooterMotor.TOP, top);
    shooter.setTargetRpm(ShooterSubsystem.ShooterMotor.MID, mid);
    shooter.setTargetRpm(ShooterSubsystem.ShooterMotor.BOTTOM, bot);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}