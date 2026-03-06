package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * PathPlanner autonomous chooser.
 * If AutoBuilder is not configured yet, this returns a safe fallback chooser
 * so robot code still runs and NetworkTables stays up.
 */
public final class Autos {
  private Autos() {}

  public static SendableChooser<Command> buildChooser() {
    SendableChooser<Command> fallback = new SendableChooser<>();
    fallback.setDefaultOption("Do Nothing", Commands.none());

    try {
      return AutoBuilder.buildAutoChooser();
    } catch (Exception e) {
      DriverStation.reportError(
          "AutoChooser build failed, AutoBuilder not configured. Check PathPlanner GUI settings files in deploy/pathplanner. " + e.getMessage(),
          e.getStackTrace()
      );
      return fallback;
    }
  }
}