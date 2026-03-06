package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterPercent extends InstantCommand {
  public SetShooterPercent(ShooterSubsystem shooter, double percent) {
    super(() -> shooter.setAllPercent(percent), shooter);
  }
}