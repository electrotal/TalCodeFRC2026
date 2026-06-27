package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.util.AngleMath;
import frc.robot.util.MultiTurnAbsoluteEncoder;

/**
 * Hood subsystem. Motor: NEO 1.1 on Spark MAX (CAN {@link Constants.CanId#kHoodAngleNeo}).
 * Position: REV Through-Bore ABSOLUTE encoder on the Spark MAX data port (getAbsoluteEncoder()),
 * wrapped for multi-turn continuity. Position units are "hood rotations": 0 = fully closed.
 *
 * <p>Control lives entirely in {@link #periodic()} (single owner of motor output): commands set
 * intent via {@link #setHoodRot(double)} (PID to a target) or {@link #setSpeed(double)} (manual
 * jog). PID output is clamped and slew-limited so it stays gentle on the mechanism.
 *
 * <p>Bring-up: drive to the closed stop with manual jog, then press Hood/ZeroNow. Re-zero after
 * every power cycle (an absolute encoder only knows its within-one-turn position at boot, and the
 * hood spans more than one encoder turn). Positive output must INCREASE Hood/HoodRot (= opening);
 * if it decreases, flip kHoodInverted (motor) or kAbsEncoderInverted (encoder) until they agree.
 */
public class HoodSubsystem extends SubsystemBase {

  // ── Hardware ────────────────────────────────────────────────────────────────

  // private final PIDController HoodPid =
  //     new PIDController(
  //         Constants.HoodConstants.kAngleP,
  //         Constants.HoodConstants.kAngleI,
  //         Constants.HoodConstants.kAngleD);

  private final SparkMax motor =
      new SparkMax(Constants.CanId.kHoodAngleNeo, SparkLowLevel.MotorType.kBrushless);

  private final RelativeEncoder encoder;

  // ── Tunable state ────────────────────────────────────────────────────────────

  private double encoderOffset = Constants.HoodConstants.kEncoderOffsetRot;
  private double setpoint = 0.1;

  // ─────────────────────────────────────────────────────────────────────────────

  public HoodSubsystem() {
    // SparkMaxConfig cfg = new SparkMaxConfig();
    // HoodPid.setTolerance(HoodConstants.hoodTolerance);
    // cfg.idleMode(SparkBaseConfig.IdleMode.kBrake);
    // cfg.inverted(Constants.MotorInverts.kHoodInverted);
    // motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    encoder = motor.getEncoder();

    SmartDashboard.putNumber("Hood/EncoderOffset", offset);
    SmartDashboard.putBoolean("Hood/ZeroNow", false);
    // Bench/Elastic angle command: type a target into Hood/TuneTargetRot, flip Hood/GoToTune on,
    // and the hood PIDs there — tune gains and drive to any set angle without a tag in view.
    SmartDashboard.putNumber("Hood/TuneTargetRot", targetHoodRot);
    SmartDashboard.putBoolean("Hood/GoToTune", false);
  }

  // ── Position ─────────────────────────────────────────────────────────────────

  /** Continuous multi-turn encoder reading, in encoder rotations. */
  public double getRawEncoder() {
    return hoodEncoder.getContinuousRot();
  }

  public double getOffsetEncoder() {
    return getRawEncoder() - offset;
  }

  /** Hood position in hood rotations (0 = closed). */
  public double getHoodRot() {
    return getOffsetEncoder() / Constants.HoodConstants.kEncoderTurnsPerHoodTurn;
  }

  /** 0 = fully closed, 100 = fully open. */
  public double getHoodPercent() {
    double range = Constants.HoodConstants.kOpenHoodRot - Constants.HoodConstants.kMinHoodRot;
    if (range == 0) return 0.0;
    return ((getHoodRot() - Constants.HoodConstants.kMinHoodRot) / range) * 100.0;
  }

  public double getTargetHoodRot() {
    return targetHoodRot;
  }

  /** True only once the hood has been zeroed this boot AND is within tolerance of its target. */
  public boolean isAtAngle() {
    return zeroed && Math.abs(getHoodRot() - targetHoodRot) <= Constants.HoodConstants.kToleranceHoodRot;
  }

  // ── Intent ───────────────────────────────────────────────────────────────────

  /** PID the hood to a target (clamped to limits). Cancels any manual jog. */
  public void setHoodRot(double rot) {
    manualPercent = 0.0;
    targetHoodRot = clampToLimits(rot);
  }

  /** Manual jog (LT/RT). 0 releases back to holding the current position. */
  public void setSpeed(double speed) {
    if (getHoodPercent() >= 100 && speed > 0) {
      stop();
      return;
    }
    motor.set(speed);
  }

  public void stop() {
    motor.set(0.0);
  }

  public void setHoodPercent(double percent)
  {
    setpoint = percent;
    AngleMath.clamp(setpoint, 0, 100);
  }

  // ── Control (single owner of motor output) ─────────────────────────────────────

  private double clampToLimits(double rot) {
    return MathUtil.clamp(rot, Constants.HoodConstants.kMinHoodRot, Constants.HoodConstants.kOpenHoodRot);
  }

  /** Stop driving into a hard limit. Positive output = opening (increasing hood rotations). */
  private double applyLimitGuard(double output) {
    double pos = getHoodRot();
    if (output > 0 && pos >= Constants.HoodConstants.kOpenHoodRot) return 0.0;
    if (output < 0 && pos <= Constants.HoodConstants.kMinHoodRot) return 0.0;
    return output;
  }

  /** Cap how fast the output can change per loop — gentle on the mechanism. */
  private double applySlew(double output) {
    double maxDelta = Constants.HoodConstants.kSlewPerLoop;
    output = MathUtil.clamp(output, lastOutput - maxDelta, lastOutput + maxDelta);
    lastOutput = output;
    return output;
  }

  @Override
  public void periodic() {
    // System.out.println(getRawEncoder());
    SmartDashboard.putNumber("Hood/RawEncoder",        getRawEncoder());
    // SmartDashboard.putNumber("Hood/OffsetEncoder",     getOffsetEncoder());
    // SmartDashboard.putNumber("Hood/HoodRot",           getHoodRot());
    SmartDashboard.putNumber("Hood/PercentOpen",       getHoodPercent());
    // SmartDashboard.putNumber("Hood/P", HoodPid.getP());
    // SmartDashboard.putNumber("Hood/I", HoodPid.getI());
    // SmartDashboard.putNumber("Hood/D", HoodPid.getD());
    // motor.set(HoodPid.calculate(getHoodPercent(), setpoint));

  }
}
