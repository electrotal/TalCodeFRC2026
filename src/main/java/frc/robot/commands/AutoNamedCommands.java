package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransportSubsystem;

/**
 * Named commands for PathPlanner event markers.
 * The names MUST match exactly what you type in the PathPlanner GUI.
 */
public final class AutoNamedCommands {
  private AutoNamedCommands() {}

  public static void register(
      SwerveSubsystem swerve,
      ShooterSubsystem shooter,
      TransportSubsystem transport,
      IntakeSubsystem intake,
      ClimberSubsystem climber) {

    // Shooter
    NamedCommands.registerCommand("StopShooter", new StopShooter(shooter));
    NamedCommands.registerCommand("SpinUpShooterDefault", new SpinShooterDefaults(shooter));

    // Feed, driver-style feed command but usable in auto too
    NamedCommands.registerCommand("FeedShooter", new FeedShooterWithIntakeJerk(transport, intake));

    // Intake
    NamedCommands.registerCommand("ToggleIntake", new ToggleIntake(intake));

    // Climber
    NamedCommands.registerCommand("ClimbUp", new ClimbUp(climber));
    NamedCommands.registerCommand("ClimbDown", new ClimbDown(climber));

    // Aim to your fixed heading under hub for autonomous (you set the angle constant on robot)
    NamedCommands.registerCommand("AimUnderHub", AimToFixedHeading.underHub(swerve));
  }
}