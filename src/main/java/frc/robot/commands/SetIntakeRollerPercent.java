package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class SetIntakeRollerPercent extends InstantCommand {

  public SetIntakeRollerPercent(IntakeSubsystem intake, DoubleSupplier percentSupplier) {
    super(() -> intake.setRollerPercent(percentSupplier.getAsDouble()), intake);
  }

  public SetIntakeRollerPercent(IntakeSubsystem intake, double percent) {
    this(intake, () -> percent);
  }
}