package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransportSubsystem;

public class FeedShooterWithIntakeJerk extends Command {

  private final TransportSubsystem transport;
  private final IntakeSubsystem intake;

  private final Timer timer = new Timer();
  private boolean forwardPhase = true;

  public FeedShooterWithIntakeJerk(TransportSubsystem transport, IntakeSubsystem intake) {
    this.transport = transport;
    this.intake = intake;
    addRequirements(transport, intake);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    forwardPhase = true;

    transport.runAll(Constants.FeedConstants.kTransportPercent);
    intake.setRollerPercent(Constants.FeedConstants.kJerkForwardPercent);
  }

  @Override
  public void execute() {
    double t = timer.get();

    if (forwardPhase && t >= Constants.FeedConstants.kJerkForwardSeconds) {
      forwardPhase = false;
      timer.reset();
      intake.setRollerPercent(Constants.FeedConstants.kJerkReversePercent);
    } else if (!forwardPhase && t >= Constants.FeedConstants.kJerkReverseSeconds) {
      forwardPhase = true;
      timer.reset();
      intake.setRollerPercent(Constants.FeedConstants.kJerkForwardPercent);
    }

    transport.runAll(Constants.FeedConstants.kTransportPercent);
  }

  @Override
  public void end(boolean interrupted) {
    transport.stopAll();

    // Only stop intake roller if intake is closed, otherwise keep behavior consistent with open state
    if (!intake.isOpen()) {
      intake.stopRoller();
    } else {
      intake.setRollerPercent(Constants.IntakeConstants.kRollerPercent);
    }

    timer.stop();
  } 

  @Override
  public boolean isFinished() {
    return false;
  }
}