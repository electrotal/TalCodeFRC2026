package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class StopShooter extends InstantCommand {
  public StopShooter(ShooterSubsystem shooter) {
    super(shooter::stop, shooter);
  }
}