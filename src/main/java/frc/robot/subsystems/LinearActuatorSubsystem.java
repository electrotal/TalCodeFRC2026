package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.AngleMath;

/**
 * Simple PWM control for an RC-style linear actuator (Actuonix L16-R series).
 * Position is normalized [0,1] like a servo command.
 */
public class LinearActuatorSubsystem extends SubsystemBase {

  private final Servo actuator = new Servo(Constants.LinearActuatorConstants.kPwmPort);

  private double closedPos = Constants.LinearActuatorConstants.kClosedPos;
  private double halfOpenPos = Constants.LinearActuatorConstants.kHalfOpenPos;
  private double commandPos = closedPos;

  public LinearActuatorSubsystem() {
    // Actuonix RC control expects 1000-2000 us style pulses.
    actuator.setBoundsMicroseconds(
        Constants.LinearActuatorConstants.kMaxPulseUs,
        0,
        Constants.LinearActuatorConstants.kCenterPulseUs,
        0,
        Constants.LinearActuatorConstants.kMinPulseUs);

    SmartDashboard.putNumber("Actuator/ClosedPos", closedPos);
    SmartDashboard.putNumber("Actuator/HalfOpenPos", halfOpenPos);
    setPosition(closedPos);
  }

  private static double clamp01(double x) {
    return AngleMath.clamp(x, 0.0, 1.0);
  }

  public void setPosition(double pos) {
    commandPos = clamp01(pos);
    actuator.set(commandPos);
    SmartDashboard.putNumber("Actuator/CmdPos", commandPos);
  }

  public void moveClosed() {
    setPosition(closedPos);
  }

  public void moveHalfOpen() {
    setPosition(halfOpenPos);
  }

  @Override
  public void periodic() {
    closedPos = clamp01(SmartDashboard.getNumber("Actuator/ClosedPos", closedPos));
    halfOpenPos = clamp01(SmartDashboard.getNumber("Actuator/HalfOpenPos", halfOpenPos));

    SmartDashboard.putNumber("Actuator/CmdPos", commandPos);
    SmartDashboard.putNumber("Actuator/CmdPulseUs", actuator.getPulseTimeMicroseconds());
    SmartDashboard.putNumber("Actuator/PwmPort", Constants.LinearActuatorConstants.kPwmPort);
  }
}
