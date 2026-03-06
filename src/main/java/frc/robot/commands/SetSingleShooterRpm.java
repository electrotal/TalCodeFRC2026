package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class SetSingleShooterRpm extends InstantCommand {

  public SetSingleShooterRpm(ShooterSubsystem shooter, ShooterSubsystem.ShooterMotor which, double rpm) {
    super(() -> shooter.setTargetRpm(which, rpm), shooter);
  }
}