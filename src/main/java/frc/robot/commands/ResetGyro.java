package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetGyro extends InstantCommand {
  public ResetGyro(SwerveSubsystem swerve) {
    super(swerve::zeroGyro, swerve);
  }
}