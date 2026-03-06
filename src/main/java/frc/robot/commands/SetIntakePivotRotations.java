package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class SetIntakePivotRotations extends InstantCommand {

  public SetIntakePivotRotations(IntakeSubsystem intake, DoubleSupplier rotationsSupplier) {
    super(() -> intake.setPivotTargetRot(rotationsSupplier.getAsDouble()), intake);
  }

  public SetIntakePivotRotations(IntakeSubsystem intake, double rotations) {
    this(intake, () -> rotations);
  }
}