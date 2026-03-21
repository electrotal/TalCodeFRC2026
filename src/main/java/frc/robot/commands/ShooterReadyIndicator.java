package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Rumbles the controller once for a short burst when the shooter first reaches
 * its target speed.  The rumble does not repeat if the PID wobbles in and out
 * of tolerance — it only fires again after the shooter has been fully stopped
 * and re-spun.
 */
public class ShooterReadyIndicator extends Command {

  private static final double kRumbleDurationSec = 0.4;

  private final ShooterSubsystem shooter;
  private final GenericHID hid;

  private boolean hasRumbled = false;   // latched true once we fire the burst
  private double rumbleStartSec = -1.0; // timestamp when burst began

  public ShooterReadyIndicator(ShooterSubsystem shooter, GenericHID hid) {
    this.shooter = shooter;
    this.hid = hid;
    addRequirements(); // no subsystem requirements — must never interrupt anything
  }

  @Override
  public void execute() {
    // Reset latch when the shooter stops so next spin-up rumbles again
    if (!shooter.isRunning()) {
      hasRumbled = false;
      rumbleStartSec = -1.0;
      setRumble(0.0);
      return;
    }

    // Trigger the burst the first time we hit target speed this cycle
    if (shooter.isAtSpeed() && !hasRumbled) {
      hasRumbled = true;
      rumbleStartSec = Timer.getFPGATimestamp();
    }

    // Drive rumble only during the burst window
    boolean inBurst = hasRumbled
        && rumbleStartSec >= 0.0
        && (Timer.getFPGATimestamp() - rumbleStartSec) < kRumbleDurationSec;
    setRumble(inBurst ? 1.0 : 0.0);
  }

  @Override
  public void end(boolean interrupted) {
    setRumble(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void setRumble(double strength) {
    hid.setRumble(GenericHID.RumbleType.kLeftRumble,  strength);
    hid.setRumble(GenericHID.RumbleType.kRightRumble, strength);
  }
}
