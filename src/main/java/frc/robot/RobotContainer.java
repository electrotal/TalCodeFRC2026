package frc.robot;

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
import frc.robot.commands.DriveFaceVirtualGoal;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.RunTransportWhileHeld;
import frc.robot.commands.ToggleIntake;
import frc.robot.util.FieldTargetUtil;
import frc.robot.util.ShotMap;
import frc.robot.subsystems.ClimberSubsystem;
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
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();

  private final DriverDisplaySubsystem display =
      new DriverDisplaySubsystem(drivebase, shooter, hood, intake, transport, climber, vision);

  private final VisionFusionSubsystem visionFusion = new VisionFusionSubsystem(drivebase, vision);

  private final CommandXboxController driver =
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

    AutoNamedCommands.register(drivebase, shooter, transport, intake, climber);

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

  private void configureBindings() {
    Trigger menuAndView = driver.start().and(driver.back());
    menuAndView.onTrue(new ResetGyro(drivebase));

    driver.x().toggleOnTrue(
        RunTransportWhileHeld.create(transport, transport::getTransportPercent)
    );

    driver.rightBumper().onTrue(new ToggleIntake(intake));
    driver.leftBumper().onTrue(Commands.runOnce(intake::togglePivotHoldEnabled));

    // Shooter wheel test toggles — RPMs tunable live via SmartDashboard/Elastic
    driver.y().toggleOnTrue(createShooterToggleCommand(shooter::getLiveLowRpm));
    driver.b().toggleOnTrue(createShooterToggleCommand(shooter::getLiveHighRpm));

    // A: run transport in reverse while held
    driver.a().whileTrue(
        Commands.runEnd(
            () -> transport.runAll(-transport.getTransportPercent()),
            transport::stopAll,
            transport));


    // Drive + auto-rotate to hub — translation stays field-oriented, rotation locks on target
    driver.leftStick().toggleOnTrue(
        new frc.robot.commands.DriveFaceHub(
            drivebase,
            () -> scaledDriveInput(-driver.getLeftY()),
            () -> scaledDriveInput(-driver.getLeftX())
        )
    );

    // Drive + auto-rotate to virtual goal (shooting on the move)
    driver.rightStick().toggleOnTrue(
        new DriveFaceVirtualGoal(
            drivebase,
            () -> scaledDriveInput(-driver.getLeftY()),
            () -> scaledDriveInput(-driver.getLeftX())
        )
    );

    // D-pad Up: toggle distance-based shooter RPM (continuously updates from robot position)
    // Also publishes Shot/HoodRot for future hood integration
    driver.povUp().toggleOnTrue(
        Commands.runEnd(
            () -> {
              double dist = FieldTargetUtil.distanceToHubMeters(drivebase.getPose());
              ShotMap.ShotSolution shot = ShotMap.calculate(dist);
              shooter.setTargetRpms(shot.topRpm(), shot.midRpm(), shot.bottomRpm());
              SmartDashboard.putNumber("Shot/DistanceM", shot.distanceMeters());
              SmartDashboard.putNumber("Shot/HoodRot", shot.hoodRot());
            },
            shooter::stop,
            shooter)
    );

    // D-pad Down: stop shooter immediately (zero voltage, no PID to zero)
    driver.povDown().onTrue(Commands.runOnce(shooter::stop, shooter));

    // Climber bindings — uncomment when climber is ready
    // D-pad Left: toggle climber down (close)
    // driver.povLeft().toggleOnTrue(
    //     Commands.startEnd(
    //         () -> climber.setPercent(Constants.ClimberConstants.kClimbDownPercent),
    //         climber::stop,
    //         climber));
    // D-pad Right: toggle climber up (open)
    // driver.povRight().toggleOnTrue(
    //     Commands.startEnd(
    //         () -> climber.setPercent(Constants.ClimberConstants.kClimbUpPercent),
    //         climber::stop,
    //         climber));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}