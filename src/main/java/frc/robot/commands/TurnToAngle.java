package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

public class TurnToAngle extends Command {

  private final SwerveSubsystem swerve;
  private final Supplier<Rotation2d> targetHeadingSupplier;
  private final PIDController controller;

  public TurnToAngle(SwerveSubsystem swerve, Supplier<Rotation2d> targetHeadingSupplier) {
    this.swerve = swerve;
    this.targetHeadingSupplier = targetHeadingSupplier;

    controller = new PIDController(
        Constants.SwerveConstants.kTurnP,
        Constants.SwerveConstants.kTurnI,
        Constants.SwerveConstants.kTurnD
    );
    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(Math.toRadians(Constants.SwerveConstants.kTurnToleranceDeg));

    addRequirements(swerve);
  }

  public TurnToAngle(SwerveSubsystem swerve, Rotation2d targetHeading) {
    this(swerve, () -> targetHeading);
  }

  @Override
  public void initialize() {
    controller.reset();
  }

  @Override
  public void execute() {
    Rotation2d current = swerve.getHeading();
    Rotation2d target = targetHeadingSupplier.get();

    double omega = controller.calculate(current.getRadians(), target.getRadians());

    double maxOmega = Constants.SwerveConstants.kMaxAngularSpeedRadPerSec;
    if (omega > maxOmega) omega = maxOmega;
    if (omega < -maxOmega) omega = -maxOmega;

    swerve.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, omega));
  }

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}