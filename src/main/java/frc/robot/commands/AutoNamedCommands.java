package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.util.FieldTargetUtil;

/**
 * Named commands for PathPlanner event markers. The names MUST match exactly what you type in the
 * PathPlanner GUI. Building blocks for autos: take → move → intake → shoot.
 */
public final class AutoNamedCommands {
  private AutoNamedCommands() {}

  public static void register(
      SwerveSubsystem swerve,
      ShooterSubsystem shooter,
      TransportSubsystem transport,
      IntakeSubsystem intake,
      HoodSubsystem hood) {

    // Shooter
    NamedCommands.registerCommand("StopShooter", new StopShooter(shooter));
    NamedCommands.registerCommand("SpinUpShooterDefault", new SpinShooterDefaults(shooter));

    // Intake
    NamedCommands.registerCommand("ToggleIntake", new ToggleIntake(intake));
    NamedCommands.registerCommand("OpenIntake", new SetIntakeOpen(intake));
    NamedCommands.registerCommand("CloseIntake", new SetIntakeClosed(intake));

    // Feed (jerks the intake while running transport) — usable in teleop and auto
    NamedCommands.registerCommand("FeedShooter", new FeedShooterWithIntakeJerk(transport, intake));

    // Prepare shot from live distance to hub (sets hood + RPM continuously)
    NamedCommands.registerCommand(
        "PrepareFromDistance",
        new UpdateShotSetpointsFromDistance(
            shooter, hood, () -> FieldTargetUtil.distanceToHubMeters(swerve.getPose())));

    // Prepare a fixed point-blank hub shot (static angle + RPM)
    NamedCommands.registerCommand("PrepareAtHub", PrepareToShootAtHub.create(shooter, hood));

    // One-shot point-blank: hold the hub preset (hood + RPM), wait until at speed, then feed.
    // Drop this on a path right next to the hub and it shoots — no aiming, no distance needed.
    NamedCommands.registerCommand(
        "ShootAtHub",
        PrepareToShootAtHub.create(shooter, hood).raceWith(
            new WaitForShooterReady(shooter).withTimeout(2.0)
                .andThen(new FeedShooterWithIntakeJerk(transport, intake).withTimeout(1.5))));

    // Full shoot: spin up, wait until at speed, then feed
    NamedCommands.registerCommand(
        "Shoot",
        new SpinShooterDefaults(shooter)
            .andThen(new WaitForShooterReady(shooter).withTimeout(2.0))
            .andThen(new FeedShooterWithIntakeJerk(transport, intake).withTimeout(1.5)));

    // Stop everything in the shooting path
    NamedCommands.registerCommand(
        "StopAll",
        Commands.runOnce(
            () -> {
              shooter.stop();
              transport.stopAll();
            },
            shooter, transport));

    // Aim to a fixed heading under the hub (you set the angle constant on the robot)
    NamedCommands.registerCommand("AimUnderHub", AimToFixedHeading.underHub(swerve));
  }
}
