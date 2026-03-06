package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.TransportSubsystem;

import java.util.function.DoubleSupplier;

public final class RunTransportWhileHeld {

  private RunTransportWhileHeld() {}

  public static Command create(TransportSubsystem transport, DoubleSupplier percentSupplier) {
    return Commands.run(
        () -> transport.runAll(percentSupplier.getAsDouble()),
        transport
    ).finallyDo(transport::stopAll);
  }

  public static Command create(TransportSubsystem transport, double percent) {
    return create(transport, () -> percent);
  }
}