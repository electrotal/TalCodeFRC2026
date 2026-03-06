package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.TransportSubsystem;

/** Runs only the main conveyor while held. */
public final class RunConveyorOnlyWhileHeld {
  private RunConveyorOnlyWhileHeld() {}

  public static Command create(TransportSubsystem transport, double percent) {
    return Commands.startEnd(
        () -> transport.runConveyorOnly(percent),
        transport::stopAll,
        transport
    );
  }
}