package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.ShotMap;

import java.util.function.DoubleSupplier;

/**
 * Continuously drives hood angle + shooter RPM from a distance-to-shot lookup. The distance is
 * supplied by the caller, so the same command serves the hub-distance follow (POV-Up) and the
 * virtual-goal distance for shooting on the move. Hood targets are hood rotations; shooter RPM.
 */
public class UpdateShotSetpointsFromDistance extends Command {

  private final ShooterSubsystem shooter;
  private final HoodSubsystem hood;
  private final DoubleSupplier distanceMeters;

  public UpdateShotSetpointsFromDistance(
      ShooterSubsystem shooter, HoodSubsystem hood, DoubleSupplier distanceMeters) {
    this.shooter = shooter;
    this.hood = hood;
    this.distanceMeters = distanceMeters;
    addRequirements(shooter, hood);
  }

  @Override
  public void execute() {
    double d = distanceMeters.getAsDouble();
    ShotMap.ShotSolution shot = ShotMap.calculate(d);

    hood.setHoodRot(shot.hoodRot());
    shooter.setTargetRpms(shot.topRpm(), shot.midRpm(), shot.bottomRpm());

    SmartDashboard.putNumber("Shot/DistanceMeters", d);
    SmartDashboard.putNumber("Shot/TargetHoodRot", shot.hoodRot());
    SmartDashboard.putNumber("Shot/TargetRpm", shot.topRpm());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
