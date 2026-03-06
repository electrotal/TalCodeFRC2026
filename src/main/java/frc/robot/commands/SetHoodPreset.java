package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.util.LinearInterpolation;

import java.util.function.DoubleSupplier;

/**
 * Hood presets:
 * - HOME: move to min hood position
 * - DISTANCE_BASED: use ShotLookup table (distance -> hood rotations)
 */
public class SetHoodPreset extends InstantCommand {

  public enum Preset {
    HOME,
    DISTANCE_BASED
  }

  public SetHoodPreset(HoodSubsystem hood, Preset preset, DoubleSupplier distanceMeters) {
    super(() -> {
      if (preset == Preset.HOME) {
        hood.setHoodRot(Constants.HoodConstants.kMinHoodRot);
        return;
      }

      double d = distanceMeters.getAsDouble();
      double hoodRot = LinearInterpolation.lookup(
          Constants.ShotLookup.kDistanceM,
          Constants.ShotLookup.kHoodRot,
          d
      );
      hood.setHoodRot(hoodRot);
    }, hood);
  }
}