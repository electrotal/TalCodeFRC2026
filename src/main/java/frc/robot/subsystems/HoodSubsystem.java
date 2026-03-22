package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.AngleMath;

/**
 * Hood:
 * - Motor: NEO 1.1 on Spark Max (CAN ID kHoodAngleNeo)
 * - Sensor: Absolute encoder on DIO 3 (duty cycle), single-turn [0, 1)
 *
 * Position zero is the lowest (fully closed) position, set via encoderOffset.
 * All positions are in offsetted encoder units (raw - offset).
 * closedPos and openPos define the full range; setHoodRot() clamps to that range.
 */
public class HoodSubsystem extends SubsystemBase {

  private final SparkMax motor =
      new SparkMax(Constants.CanId.kHoodAngleNeo, SparkLowLevel.MotorType.kBrushless);

  private final DutyCycleEncoder absEncoder =
      new DutyCycleEncoder(Constants.HoodConstants.kThroughBoreDio);

  private final PIDController pid =
      new PIDController(Constants.HoodConstants.kP, Constants.HoodConstants.kI, Constants.HoodConstants.kD);

  // All tunable live from Elastic
  private double encoderOffset = Constants.HoodConstants.kEncoderOffsetRot;
  private double closedPos     = Constants.HoodConstants.kClosedPos;
  private double openPos       = Constants.HoodConstants.kOpenPos;
  private double maxOut        = Constants.HoodConstants.kMaxOut;

  // Target in offsetted encoder units
  private double targetPos = Constants.HoodConstants.kClosedPos;

  public HoodSubsystem() {
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.idleMode(SparkBaseConfig.IdleMode.kBrake);
    cfg.inverted(Constants.MotorInverts.kHoodAngleInverted);
    motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pid.setTolerance(Constants.HoodConstants.kToleranceHoodRot);

    SmartDashboard.putNumber("Hood/EncoderOffset", encoderOffset);
    SmartDashboard.putNumber("Hood/ClosedPos",     closedPos);
    SmartDashboard.putNumber("Hood/OpenPos",       openPos);
    SmartDashboard.putNumber("Hood/MaxOut",        maxOut);
    SmartDashboard.putNumber("Hood/TargetPos",     targetPos);
    SmartDashboard.putNumber("Hood/PID/kP",        Constants.HoodConstants.kP);
    SmartDashboard.putNumber("Hood/PID/kI",        Constants.HoodConstants.kI);
    SmartDashboard.putNumber("Hood/PID/kD",        Constants.HoodConstants.kD);
  }

  /** Raw absolute encoder value [0, 1). */
  public double getRawEncoder() {
    return absEncoder.get();
  }

  /** Position with offset applied — 0.0 = lowest (fully closed). */
  public double getPosition() {
    return getRawEncoder() - encoderOffset;
  }

  /** Alias for DriverDisplaySubsystem / legacy commands. */
  public double getHoodRot() {
    return getPosition();
  }

  public double getTargetHoodRot() {
    return targetPos;
  }

  /**
   * Set target position in offsetted encoder units.
   * Clamped to [closedPos, openPos] (or reversed if openPos < closedPos).
   */
  public void setHoodRot(double pos) {
    double lo = Math.min(closedPos, openPos);
    double hi = Math.max(closedPos, openPos);
    targetPos = AngleMath.clamp(pos, lo, hi);
    SmartDashboard.putNumber("Hood/TargetPos", targetPos);
  }

  public void setFullyClosed() {
    setHoodRot(closedPos);
  }

  public void setFullyOpen() {
    setHoodRot(openPos);
  }

  public boolean atTarget() {
    return pid.atSetpoint();
  }

  public void stop() {
    motor.set(0.0);
  }

  @Override
  public void periodic() {
    // Read tunables from Elastic
    encoderOffset = SmartDashboard.getNumber("Hood/EncoderOffset", encoderOffset);
    closedPos     = SmartDashboard.getNumber("Hood/ClosedPos",     closedPos);
    openPos       = SmartDashboard.getNumber("Hood/OpenPos",       openPos);
    maxOut        = SmartDashboard.getNumber("Hood/MaxOut",        maxOut);
    targetPos     = SmartDashboard.getNumber("Hood/TargetPos",     targetPos);

    double newP = SmartDashboard.getNumber("Hood/PID/kP", pid.getP());
    double newI = SmartDashboard.getNumber("Hood/PID/kI", pid.getI());
    double newD = SmartDashboard.getNumber("Hood/PID/kD", pid.getD());
    if (newP != pid.getP() || newI != pid.getI() || newD != pid.getD()) {
      pid.setPID(newP, newI, newD);
    }

    double pos = getPosition();
    double out = pid.calculate(pos, targetPos);
    out = AngleMath.clamp(out, -maxOut, maxOut);
    motor.set(out);

    SmartDashboard.putNumber("Hood/RawEncoder", getRawEncoder());
    SmartDashboard.putNumber("Hood/Position",   pos);
    SmartDashboard.putNumber("Hood/TargetPos",  targetPos);
    SmartDashboard.putNumber("Hood/Error",      targetPos - pos);
    SmartDashboard.putBoolean("Hood/AtTarget",  atTarget());
  }
}
