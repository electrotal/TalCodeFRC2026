package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Drives the hood to a target position (in hood rotations) using the subsystem's
 * closed-loop PID. Runs while scheduled so it keeps holding the target.
 *
 * Two flavours:
 *  - fixed target: new SetHoodAngle(hood, 0.6)
 *  - live target:  new SetHoodAngle(hood, () -> SmartDashboard.getNumber("Hood/TuneTargetRot", 0))
 *    (used for on-the-fly tuning — sweep the target from Elastic while watching it track).
 */
public class SetHoodAngle extends Command {

  private final HoodSubsystem hood;
  private final DoubleSupplier targetRot;

  public SetHoodAngle(HoodSubsystem hood, double targetRot) {
    this(hood, () -> targetRot);
  }

  public SetHoodAngle(HoodSubsystem hood, DoubleSupplier targetRot) {
    this.hood = hood;
    this.targetRot = targetRot;
    addRequirements(hood);
  }

  @Override
  public void execute() {
    hood.setHoodPosition(targetRot.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    hood.stop();
  }
}
