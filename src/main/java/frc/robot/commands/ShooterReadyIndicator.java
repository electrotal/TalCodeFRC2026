package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterReadyIndicator extends Command {

  private final ShooterSubsystem shooter;
  private final GenericHID hid;

  public ShooterReadyIndicator(ShooterSubsystem shooter, GenericHID hid) {
    this.shooter = shooter;
    this.hid = hid;
    addRequirements(); // no subsystem requirements, it should not interrupt anything
  }

  @Override
  public void execute() {
    boolean ready = shooter.isAtSpeed();
    double rumble = ready ? 1.0 : 0.0;
    hid.setRumble(GenericHID.RumbleType.kLeftRumble, rumble);
    hid.setRumble(GenericHID.RumbleType.kRightRumble, rumble);
  }

  @Override
  public void end(boolean interrupted) {
    hid.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
    hid.setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}