package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.FieldTargetUtil;

import java.util.function.DoubleSupplier;

public class DriveFaceHub extends Command {

  private final Command inner;

  public DriveFaceHub(SwerveSubsystem swerve, DoubleSupplier xFieldMetersPerSec, DoubleSupplier yFieldMetersPerSec) {
    inner =
        new DriveFacePoint(
            swerve,
            xFieldMetersPerSec,
            yFieldMetersPerSec,
            FieldTargetUtil::hubCenterForAlliance
        );

    addRequirements(swerve);
  }

  @Override
  public void initialize() { inner.initialize(); }

  @Override
  public void execute() { inner.execute(); }

  @Override
  public void end(boolean interrupted) { inner.end(interrupted); }

  @Override
  public boolean isFinished() { return false; }
}