package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveHoldHeading extends Command {

  private final SwerveSubsystem swerve;

  // Desired field-relative translation, in meters per second
  private final DoubleSupplier xFieldMetersPerSec;
  private final DoubleSupplier yFieldMetersPerSec;

  // Desired robot heading (field angle)
  private final Supplier<Rotation2d> targetHeading;

  private final PIDController headingPid;

  public DriveHoldHeading(
      SwerveSubsystem swerve,
      DoubleSupplier xFieldMetersPerSec,
      DoubleSupplier yFieldMetersPerSec,
      Supplier<Rotation2d> targetHeading) {

    this.swerve = swerve;
    this.xFieldMetersPerSec = xFieldMetersPerSec;
    this.yFieldMetersPerSec = yFieldMetersPerSec;
    this.targetHeading = targetHeading;

    headingPid = new PIDController(
        Constants.SwerveConstants.kHoldHeadingP,
        Constants.SwerveConstants.kHoldHeadingI,
        Constants.SwerveConstants.kHoldHeadingD);

    headingPid.enableContinuousInput(-Math.PI, Math.PI);
    headingPid.setTolerance(Math.toRadians(Constants.SwerveConstants.kTurnToleranceDeg));

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    headingPid.reset();
  }

  @Override
  public void execute() {
    Rotation2d currentHeading = swerve.getHeading();
    Rotation2d target = targetHeading.get();

    double omega = headingPid.calculate(currentHeading.getRadians(), target.getRadians());

    double maxOmega = Constants.SwerveConstants.kMaxAngularSpeedRadPerSec;
    if (omega > maxOmega) omega = maxOmega;
    if (omega < -maxOmega) omega = -maxOmega;

    double xField = xFieldMetersPerSec.getAsDouble();
    double yField = yFieldMetersPerSec.getAsDouble();

    // Convert field-relative translation to robot-relative, then let YAGSL handle field-oriented driving.
    ChassisSpeeds fieldRelative =
        ChassisSpeeds.fromFieldRelativeSpeeds(xField, yField, omega, currentHeading);

    swerve.driveRobotRelative(fieldRelative);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}