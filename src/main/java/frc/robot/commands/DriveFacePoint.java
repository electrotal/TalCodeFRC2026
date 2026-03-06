package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.AimingUtil;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveFacePoint extends Command {

  private final DriveHoldHeading holdHeading;

  public DriveFacePoint(
      SwerveSubsystem swerve,
      DoubleSupplier xFieldMetersPerSec,
      DoubleSupplier yFieldMetersPerSec,
      Supplier<Translation2d> targetPointSupplier) {

    Supplier<Rotation2d> targetHeading =
        () -> AimingUtil.headingToPoint(swerve.getPose(), targetPointSupplier.get());

    holdHeading = new DriveHoldHeading(
        swerve,
        xFieldMetersPerSec,
        yFieldMetersPerSec,
        targetHeading);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    holdHeading.initialize();
  }

  @Override
  public void execute() {
    holdHeading.execute();
  }

  @Override
  public void end(boolean interrupted) {
    holdHeading.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}