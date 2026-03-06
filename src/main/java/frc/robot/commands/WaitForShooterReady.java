package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class WaitForShooterReady extends Command {

  // This command only checks readiness, it should not take control of the shooter
  private final ShooterSubsystem shooter;

  public WaitForShooterReady(ShooterSubsystem shooter) {
    this.shooter = shooter;
  }

  @Override
  public boolean isFinished() {
    return shooter.isAtSpeedForTime(Constants.ShooterConstants.kReadyTimeSeconds);
  }
}