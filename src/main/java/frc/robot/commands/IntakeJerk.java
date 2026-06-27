package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Intake jerk (POV-Left): oscillates the intake PIVOT between clopen and a "half-clopen" position
 * to physically shake balls into the robot. The rollers/wheels are kept OFF the whole time.
 * Positions and dwell are tunable in {@link Constants.IntakeConstants}.
 */
public class IntakeJerk extends Command {

  private final IntakeSubsystem intake;
  private final Timer timer = new Timer();
  private boolean atClopen = true;

  public IntakeJerk(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    timer.restart();
    atClopen = true;
    intake.setPivotHoldEnabled(true);   // make sure the pivot PID is actually driving
    intake.stopRoller();                // wheels off
    intake.setPivotTargetRot(Constants.IntakeConstants.kClopenPivotRot);
  }

  @Override
  public void execute() {
    intake.stopRoller(); // keep the wheels off the whole time
    if (timer.hasElapsed(Constants.IntakeConstants.kJerkDwellSeconds)) {
      atClopen = !atClopen;
      timer.restart();
      intake.setPivotTargetRot(
          atClopen
              ? Constants.IntakeConstants.kClopenPivotRot
              : Constants.IntakeConstants.kJerkHalfClopenRot);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopRoller();
    intake.setPivotTargetRot(Constants.IntakeConstants.kClopenPivotRot); // settle back at clopen
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
