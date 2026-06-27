package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Up-close / point-blank hub preset (POV-Right): a FIXED hood angle + RPM for shooting parked right
 * at the hub — no distance lookup, no auto-aim. Both values are live-tunable on the dashboard
 * (Hub/PresetHoodRot, Hub/PresetRpm) so you can dial them in at the practice match. Holds while
 * scheduled; releasing freezes them (POV-Down stops the shooter).
 */
public final class PrepareToShootAtHub {

  private PrepareToShootAtHub() {}

  public static Command create(ShooterSubsystem shooter, HoodSubsystem hood) {
    SmartDashboard.putNumber("Hub/PresetHoodRot", Constants.HoodConstants.kHubPresetHoodRot);
    SmartDashboard.putNumber("Hub/PresetRpm", Constants.ShooterConstants.kHubPresetRpm);
    return Commands.run(
        () -> {
          hood.setHoodRot(SmartDashboard.getNumber("Hub/PresetHoodRot", Constants.HoodConstants.kHubPresetHoodRot));
          shooter.setAllTargetRpm(SmartDashboard.getNumber("Hub/PresetRpm", Constants.ShooterConstants.kHubPresetRpm));
        },
        shooter, hood);
  }
}
