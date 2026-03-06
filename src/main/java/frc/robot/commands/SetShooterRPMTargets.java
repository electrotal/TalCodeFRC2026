package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterRPMTargets extends InstantCommand {

  public SetShooterRPMTargets(
      ShooterSubsystem shooter,
      double topRpm,
      double midRpm,
      double bottomRpm) {

    super(
        () -> {
          shooter.setTargetRpm(ShooterSubsystem.ShooterMotor.TOP, topRpm);
          shooter.setTargetRpm(ShooterSubsystem.ShooterMotor.MID, midRpm);
          shooter.setTargetRpm(ShooterSubsystem.ShooterMotor.BOTTOM, bottomRpm);
        },
        shooter
    );
  }

  public static SetShooterRPMTargets defaults(ShooterSubsystem shooter) {
    return new SetShooterRPMTargets(
        shooter,
        Constants.ShooterConstants.kTopRpm,
        Constants.ShooterConstants.kMidRpm,
        Constants.ShooterConstants.kBottomRpm
    );
  }
}