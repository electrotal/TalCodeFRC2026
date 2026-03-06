package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

/** Spins shooter wheels to default RPMs. */
public class SpinShooterDefaults extends InstantCommand {
  public SpinShooterDefaults(ShooterSubsystem shooter) {
    super(() -> {
      shooter.setTargetRpm(ShooterSubsystem.ShooterMotor.TOP, Constants.ShooterConstants.kTopRpm);
      shooter.setTargetRpm(ShooterSubsystem.ShooterMotor.MID, Constants.ShooterConstants.kMidRpm);
      shooter.setTargetRpm(ShooterSubsystem.ShooterMotor.BOTTOM, Constants.ShooterConstants.kBottomRpm);
    }, shooter);
  }
}