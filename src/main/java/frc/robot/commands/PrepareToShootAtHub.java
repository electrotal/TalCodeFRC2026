package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public final class PrepareToShootAtHub {

  private PrepareToShootAtHub() {}

  public static Command create(
      SwerveSubsystem swerve,
      ShooterSubsystem shooter,
      HoodSubsystem hood,
      DoubleSupplier xField,
      DoubleSupplier yField) {

    return Commands.parallel(
        new DriveFaceHub(swerve, xField, yField),
        new UpdateShotSetpointsFromDistance(swerve, shooter, hood)
    );
  }
}