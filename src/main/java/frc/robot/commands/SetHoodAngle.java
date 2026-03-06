package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HoodSubsystem;

/**
 * Sets hood target position in HOOD rotations (based on through-bore encoder).
 * Name kept as SetHoodAngle for convenience, but units are rotations.
 */
public class SetHoodAngle extends InstantCommand {

  public SetHoodAngle(HoodSubsystem hood, double hoodRot) {
    super(() -> hood.setHoodRot(hoodRot), hood);
  }
}