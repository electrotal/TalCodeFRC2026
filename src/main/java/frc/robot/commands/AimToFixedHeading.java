package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class AimToFixedHeading extends Command {

  private final DriveHoldHeading hold;

  public AimToFixedHeading(SwerveSubsystem swerve, double headingDeg) {
    hold = new DriveHoldHeading(
        swerve,
        () -> 0.0,
        () -> 0.0,
        () -> Rotation2d.fromDegrees(headingDeg)
    );
    addRequirements(swerve);
  }

  public static AimToFixedHeading underHub(SwerveSubsystem swerve) {
    return new AimToFixedHeading(swerve, Constants.AutoAimConstants.kUnderHubHeadingDeg);
  }

  @Override
  public void initialize() { hold.initialize(); }

  @Override
  public void execute() { hold.execute(); }

  @Override
  public void end(boolean interrupted) { hold.end(interrupted); }

  @Override
  public boolean isFinished() { return false; }
}