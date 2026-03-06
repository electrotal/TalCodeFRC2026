package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.TransportSubsystem;

/** Runs only the shooter feeder while held. */
public final class RunFeederOnlyWhileHeld {
  private RunFeederOnlyWhileHeld() {}

  public static Command create(TransportSubsystem transport, double percent) {
    return Commands.startEnd(
        () -> transport.runFeederOnly(percent),
        transport::stopAll,
        transport
    );
  }
}