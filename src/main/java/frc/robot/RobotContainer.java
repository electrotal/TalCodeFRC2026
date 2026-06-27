package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoNamedCommands;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveFaceHub;
import frc.robot.commands.DriveFaceVirtualGoal;
import frc.robot.commands.IntakeJerk;
import frc.robot.commands.PrepareToShootAtHub;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.RunTransportWhileHeld;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.UpdateShotSetpointsFromDistance;
import frc.robot.util.AimingUtil;
import frc.robot.util.FieldTargetUtil;
import frc.robot.util.ShootingOnTheMoveUtil;
import frc.robot.subsystems.DriverDisplaySubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.VisionFusionSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final SwerveSubsystem drivebase = new SwerveSubsystem();

  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final TransportSubsystem transport = new TransportSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final HoodSubsystem hood = new HoodSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();

  private final DriverDisplaySubsystem display =
      new DriverDisplaySubsystem(drivebase, shooter, hood, vision);

  private final VisionFusionSubsystem visionFusion = new VisionFusionSubsystem(drivebase, vision);

  public final CommandXboxController driver =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser;

  private final SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -driver.getLeftY(),
              () -> -driver.getLeftX())
          .withControllerRotationAxis(driver::getRightX)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  private final Command teleopDrive =
      drivebase.driveFieldOriented(driveAngularVelocity);

  public RobotContainer() {
    CommandScheduler.getInstance().registerSubsystem(display, visionFusion);

    AutoNamedCommands.register(drivebase, shooter, transport, intake, hood);

    autoChooser = Autos.buildChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    drivebase.setDefaultCommand(teleopDrive);
    configureBindings();
  }

  private Command createShooterToggleCommand(DoubleSupplier rpmSupplier) {
    return Commands.startEnd(
        () -> shooter.setAllTargetRpm(rpmSupplier.getAsDouble()),
        shooter::stop,
        shooter);
  }

  /** Apply deadband and scale raw stick [-1,1] to field-relative m/s. */
  private double scaledDriveInput(double raw) {
    if (Math.abs(raw) < OperatorConstants.DEADBAND) return 0.0;
    return raw * Constants.maxSpeed * 0.8;
  }

  private static boolean isRedAlliance() {
    var a = DriverStation.getAlliance();
    return a.isPresent() && a.get() == DriverStation.Alliance.Red;
  }

  // Field-frame translation for the lock-on commands, alliance-flipped to MATCH the default drive's
  // allianceRelativeControl(true). Without this the lock-on felt robot-centric/mirrored on red
  // (driver-forward became field-backward). Front = away from the driver on both alliances.
  private double allianceFieldX() {
    double v = scaledDriveInput(-driver.getLeftY());
    return isRedAlliance() ? -v : v;
  }

  private double allianceFieldY() {
    double v = scaledDriveInput(-driver.getLeftX());
    return isRedAlliance() ? -v : v;
  }

  // Driver rotation override (rad/s) from the right stick. When the driver rotates, they win over
  // the auto-aim heading PID — the limelight is noisy during calibration, the gyro is reliable.
  private double rotationOverrideRadPerSec() {
    double rx = driver.getRightX();
    if (Math.abs(rx) < OperatorConstants.DEADBAND) return 0.0;
    return -rx * Constants.SwerveConstants.kMaxAngularSpeedRadPerSec;
  }

  private void configureBindings() {
    Trigger menuAndView = driver.start().and(driver.back());
    menuAndView.onTrue(new ResetGyro(drivebase));

    driver.x().toggleOnTrue(
        RunTransportWhileHeld.create(transport, transport::getTransportPercent));

    driver.rightBumper().onTrue(new ToggleIntake(intake));
    driver.leftBumper().onTrue(Commands.runOnce(intake::togglePivotHoldEnabled));
    driver.y().onTrue(Commands.runOnce(intake::clopen, intake));

    // B: shooter wheel test toggle — RPM tunable live via Elastic.
    driver.b().toggleOnTrue(createShooterToggleCommand(shooter::getLiveHighRpm));

    // A: run transport in reverse while held.
    driver.a().whileTrue(
        Commands.runEnd(
            () -> transport.runAll(-transport.getTransportPercent()),
            transport::stopAll,
            transport));

    // LT/RT: manual hood jog — for calibration (drive to the closed stop to zero, find the open limit).
    driver.leftTrigger().whileTrue(
        Commands.startEnd(() -> hood.setSpeed(-1.0), () -> hood.setSpeed(0), hood));
    driver.rightTrigger().whileTrue(
        Commands.startEnd(() -> hood.setSpeed(1.0), () -> hood.setSpeed(0), hood));

    // Distance suppliers for the shot map.
    DoubleSupplier hubDistance =
        () -> FieldTargetUtil.distanceToHubMeters(drivebase.getPose());
    DoubleSupplier virtualGoalDistance =
        () -> AimingUtil.distanceToPointMeters(
            drivebase.getPose(),
            ShootingOnTheMoveUtil.virtualGoalFromSwerve(drivebase, FieldTargetUtil.hubCenterForAlliance()));

    // Left stick click: SHOOT ON THE MOVE — face the virtual (leading) goal AND drive shooter RPM +
    // hood angle from that same virtual-goal distance.
    driver.leftStick().toggleOnTrue(
        new DriveFaceVirtualGoal(
                drivebase, this::allianceFieldX, this::allianceFieldY, this::rotationOverrideRadPerSec)
            .alongWith(new UpdateShotSetpointsFromDistance(shooter, hood, virtualGoalDistance)));

    // Right stick click: lock rotation to the hub (plain auto-aim; shoot via POV).
    driver.rightStick().toggleOnTrue(
        new DriveFaceHub(
            drivebase, this::allianceFieldX, this::allianceFieldY, this::rotationOverrideRadPerSec));

    // POV-Up: distance follow — ramp shooter RPM AND hood angle from live distance; holds until
    // toggled off or POV-Down.
    driver.povUp().toggleOnTrue(new UpdateShotSetpointsFromDistance(shooter, hood, hubDistance));

    // POV-Down: stop the shooter (ends the distance follow / hub preset).
    driver.povDown().onTrue(Commands.runOnce(shooter::stop, shooter));

    // POV-Right: point-blank HUB PRESET — fixed min hood angle + fixed RPM for shooting parked in
    // front of the hub, no distance tracking. Toggle off to freeze; POV-Down to stop. (Constants:
    // HoodConstants.kHubPresetHoodRot, ShooterConstants.kHubPresetRpm.)
    driver.povRight().toggleOnTrue(PrepareToShootAtHub.create(shooter, hood));

    // POV-Left: wiggle the intake PIVOT (clopen <-> half-clopen) to shake balls in. Wheels stay off.
    driver.povLeft().whileTrue(new IntakeJerk(intake));

    // Ready indicator: rumble while BOTH shooter RPM and hood angle are on target. The green light
    // is the Robot/ReadyToShoot boolean published by DriverDisplaySubsystem.
    new Trigger(display::isReadyToShoot).whileTrue(
        Commands.startEnd(
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0.4),
            () -> driver.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
