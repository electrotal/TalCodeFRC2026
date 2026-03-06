package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TransportSubsystem;

/** Stops both conveyor and feeder. */
public class StopTransport extends InstantCommand {
  public StopTransport(TransportSubsystem transport) {
    super(transport::stopAll, transport);
  }
}