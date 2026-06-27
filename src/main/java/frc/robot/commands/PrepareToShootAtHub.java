package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Point-blank hub preset: a FIXED hood angle (min/steep) and a FIXED RPM for shooting while
 * parked right in front of the hub — no distance lookup, no auto-aim. Used by POV-Right and by
 * the "PrepareAtHub" auto named command. Holds the setpoints while scheduled; releasing freezes
 * them (POV-Down stops the shooter).
 */
public final class PrepareToShootAtHub {

  private PrepareToShootAtHub() {}

  public static Command create(ShooterSubsystem shooter, HoodSubsystem hood) {
    return Commands.run(
        () -> {
          hood.setHoodRot(Constants.HoodConstants.kHubPresetHoodRot);
          shooter.setAllTargetRpm(Constants.ShooterConstants.kHubPresetRpm);
        },
        shooter, hood);
  }
}
