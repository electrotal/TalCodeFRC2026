package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoNamedCommands;
import frc.robot.commands.Autos;
// import frc.robot.commands.ClimbDown;
// import frc.robot.commands.ClimbUp;
import frc.robot.commands.DriveFaceVirtualGoal;
// import frc.robot.commands.FeedShooterWithIntakeJerk;
// import frc.robot.commands.PrepareToShootAtHub;
import frc.robot.commands.ResetGyro;
// import frc.robot.commands.RunConveyorOnlyWhileHeld;
// import frc.robot.commands.RunFeederOnlyWhileHeld;
import frc.robot.commands.RunTransportWhileHeld;
// import frc.robot.commands.ShooterReadyIndicator;
// import frc.robot.commands.StopShooter;
// import frc.robot.commands.StopTransport;
import frc.robot.commands.ToggleIntake;
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

  private void configureBindings() {
    Trigger menuAndView = driver.start().and(driver.back());
    menuAndView.onTrue(new ResetGyro(drivebase));

    // Transport toggle (X): one press starts both conveyors + shooter feeder together, next press stops
    // Live-tune speed via Transport/TransportPercent on the dashboard
    driver.x().toggleOnTrue(
        RunTransportWhileHeld.create(transport, transport::getTransportPercent)
    );

    driver.rightBumper().onTrue(new ToggleIntake(intake));

    // TESTING: Toggle intake pivot hold. When OFF, the intake pivot will not fight you.
    driver.leftBumper().onTrue(Commands.runOnce(intake::togglePivotHoldEnabled));

    // driver.y().whileTrue(
    //     PrepareToShootAtHub.create(
    //         drivebase,
    //         shooter,
    //         hood,
    //         () -> -driver.getLeftY(),
    //         () -> -driver.getLeftX()
    //     )
    // );
    // driver.y().whileTrue(new ShooterReadyIndicator(shooter, driver.getHID()));

    // driver.b().whileTrue(new FeedShooterWithIntakeJerk(transport, intake));

    // driver.a().onTrue(new StopShooter(shooter));

    // driver.povUp().whileTrue(new ClimbUp(climber));
    // driver.povDown().whileTrue(new ClimbDown(climber));

    // driver.back().onTrue(new StopTransport(transport));
    // driver.povLeft().whileTrue(RunFeederOnlyWhileHeld.create(transport, Constants.TransportConstants.kFeederOnlyPercent));
    // driver.povRight().whileTrue(RunConveyorOnlyWhileHeld.create(transport, Constants.TransportConstants.kConveyorOnlyPercent));

    driver.leftStick().toggleOnTrue(
        new frc.robot.commands.DriveFaceHub(
            drivebase,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX()
        )
    );

    driver.rightStick().toggleOnTrue(
        new DriveFaceVirtualGoal(
            drivebase,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX()
        )
    );
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}