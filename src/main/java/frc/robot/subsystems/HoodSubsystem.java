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

  private final SparkMax motor =
      new SparkMax(Constants.CanId.kHoodAngleNeo, SparkLowLevel.MotorType.kBrushless);
  private final SparkAbsoluteEncoder absEncoder = motor.getAbsoluteEncoder();
  private final MultiTurnAbsoluteEncoder hoodEncoder =
      new MultiTurnAbsoluteEncoder(absEncoder::getPosition, () -> true);

  private final PIDController pid =
      new PIDController(
          Constants.HoodConstants.kP,
          Constants.HoodConstants.kI,
          Constants.HoodConstants.kD);

  private double offset = Constants.HoodConstants.kEncoderOffsetRot;
  private double targetHoodRot = Constants.HoodConstants.kClosedHoodRot;
  private double manualPercent = 0.0;
  private double lastOutput = 0.0;
  private boolean zeroed = false;

  public HoodSubsystem() {
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.idleMode(SparkBaseConfig.IdleMode.kBrake);
    cfg.inverted(Constants.MotorInverts.kHoodInverted);
    cfg.smartCurrentLimit(30); // protect the NEO/gearbox if the hood stalls on a hard stop
    cfg.absoluteEncoder.inverted(Constants.HoodConstants.kAbsEncoderInverted);
    // CAN: the absolute-encoder position is our feedback — keep it fast; slow everything else.
    cfg.signals
        .absoluteEncoderPositionPeriodMs(20)
        .primaryEncoderPositionPeriodMs(500)
        .primaryEncoderVelocityPeriodMs(500)
        .appliedOutputPeriodMs(50)
        .busVoltagePeriodMs(500)
        .outputCurrentPeriodMs(500)
        .motorTemperaturePeriodMs(1000);
    motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pid.setTolerance(Constants.HoodConstants.kToleranceHoodRot);

    // Gains + limits: live-tunable AND persistent across reboots (stored on the roboRIO via
    // WPILib Preferences). initDouble seeds only the first time, so dialed-in values survive a
    // redeploy. (Adopted from the teammate hood commit — the right call for tuning.)
    Preferences.initDouble("Hood/kP", Constants.HoodConstants.kP);
    Preferences.initDouble("Hood/kI", Constants.HoodConstants.kI);
    Preferences.initDouble("Hood/kD", Constants.HoodConstants.kD);
    Preferences.initDouble("Hood/MaxOut", Constants.HoodConstants.kMaxOut);
    Preferences.initDouble("Hood/OpenLimitRot", Constants.HoodConstants.kOpenHoodRot);

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
    manualPercent = speed;
  }

  public void stop() {
    manualPercent = 0.0;
    targetHoodRot = clampToLimits(getHoodRot());
  }

  /** Capture the current reading as the new zero (closed stop). No hardware reset. */
  public void zeroAtCurrent() {
    offset = getRawEncoder();
    Constants.HoodConstants.kEncoderOffsetRot = offset;
    targetHoodRot = Constants.HoodConstants.kClosedHoodRot;
    zeroed = true;
    SmartDashboard.putNumber("Hood/EncoderOffset", offset);
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
    // Live-tunable gains + calibration knobs — persistent across reboots via Preferences.
    Constants.HoodConstants.kP = Preferences.getDouble("Hood/kP", Constants.HoodConstants.kP);
    Constants.HoodConstants.kI = Preferences.getDouble("Hood/kI", Constants.HoodConstants.kI);
    Constants.HoodConstants.kD = Preferences.getDouble("Hood/kD", Constants.HoodConstants.kD);
    Constants.HoodConstants.kMaxOut = Preferences.getDouble("Hood/MaxOut", Constants.HoodConstants.kMaxOut);
    Constants.HoodConstants.kOpenHoodRot =
        Preferences.getDouble("Hood/OpenLimitRot", Constants.HoodConstants.kOpenHoodRot);
    pid.setPID(Constants.HoodConstants.kP, Constants.HoodConstants.kI, Constants.HoodConstants.kD);

    // Software-zero button (captures current reading at the closed stop, then self-resets).
    if (SmartDashboard.getBoolean("Hood/ZeroNow", false)) {
      zeroAtCurrent();
      SmartDashboard.putBoolean("Hood/ZeroNow", false);
    }

    // Dashboard angle command: drive the hood to any angle typed on Elastic (calibration / tuning).
    if (SmartDashboard.getBoolean("Hood/GoToTune", false)) {
      setHoodRot(SmartDashboard.getNumber("Hood/TuneTargetRot", targetHoodRot));
    }

    SmartDashboard.putNumber("Hood/HoodRot", getHoodRot());
    SmartDashboard.putNumber("Hood/RawEncoder", getRawEncoder());
    SmartDashboard.putNumber("Hood/TargetRot", targetHoodRot);
    SmartDashboard.putBoolean("Hood/AtAngle", isAtAngle());
    SmartDashboard.putBoolean("Hood/Zeroed", zeroed);

    double maxOut = Constants.HoodConstants.kMaxOut;
    double output;
    if (manualPercent != 0.0) {
      output = MathUtil.clamp(manualPercent, -maxOut, maxOut);
      targetHoodRot = clampToLimits(getHoodRot()); // hold here when released
    } else if (zeroed) {
      output = MathUtil.clamp(pid.calculate(getHoodRot(), targetHoodRot), -maxOut, maxOut);
    } else {
      output = 0.0; // not zeroed: only manual jog allowed (drive to closed, then Hood/ZeroNow)
    }

    output = applyLimitGuard(output);
    output = applySlew(output);
    motor.set(output);
  }
}
