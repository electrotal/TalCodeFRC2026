package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MultiTurnAbsoluteEncoder;

/**
 * Hood subsystem. Motor: NEO 1.1 on Spark MAX (CAN {@link Constants.CanId#kHoodAngleNeo}).
 * Position: REV Through-Bore absolute encoder on RoboRIO DIO {@link Constants.HoodConstants#kThroughBoreDio}
 * (duty-cycle), wrapped for multi-turn continuity. Position units are "hood rotations": 0 = fully closed.
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
  // REV Through-Bore on RoboRIO DIO (duty-cycle absolute), wrapped for multi-turn continuity.
  private final DutyCycleEncoder throughBore =
      new DutyCycleEncoder(Constants.HoodConstants.kThroughBoreDio);
  private final MultiTurnAbsoluteEncoder hoodEncoder =
      new MultiTurnAbsoluteEncoder(
          () -> Constants.HoodConstants.kAbsEncoderInverted ? 1.0 - throughBore.get() : throughBore.get(),
          throughBore::isConnected);

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
    // CAN: hood feedback is the DIO through-bore, not the Spark MAX — no SparkMax status frame is
    // needed for control, so slow them all.
    cfg.signals
        .primaryEncoderPositionPeriodMs(500)
        .primaryEncoderVelocityPeriodMs(500)
        .appliedOutputPeriodMs(100)
        .busVoltagePeriodMs(500)
        .outputCurrentPeriodMs(500)
        .motorTemperaturePeriodMs(1000);
    motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pid.setTolerance(Constants.HoodConstants.kToleranceHoodRot);

    // Gains + limits: live-tunable straight from Elastic (SmartDashboard) — edit them directly.
    SmartDashboard.putNumber("Hood/kP", Constants.HoodConstants.kP);
    SmartDashboard.putNumber("Hood/kI", Constants.HoodConstants.kI);
    SmartDashboard.putNumber("Hood/kD", Constants.HoodConstants.kD);
    SmartDashboard.putNumber("Hood/MaxOut", Constants.HoodConstants.kMaxOut);
    SmartDashboard.putNumber("Hood/OpenLimitRot", Constants.HoodConstants.kOpenHoodRot);

    SmartDashboard.putNumber("Hood/EncoderOffset", offset);
    SmartDashboard.putBoolean("Hood/ZeroNow", false);
    // LT/RT manual-jog strength (open-loop duty cycle). Raise it if the hood is too weak to move.
    SmartDashboard.putNumber("Hood/JogPercent", 0.2);
    // Bench tuning: set Hood/TunePercent (0-100 % of the 0..OpenLimit range), flip Hood/GoToTune on,
    // and the hood PIDs there. Sweep the percent to see the PID track across your working range.
    SmartDashboard.putNumber("Hood/TunePercent", 0.0);
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

  /** PID the hood to a percent (0-100) of its working range [closed=0, OpenLimit]. */
  public void setHoodPercentOfRange(double percent) {
    double pct = MathUtil.clamp(percent, 0.0, 100.0) / 100.0;
    double min = Constants.HoodConstants.kMinHoodRot;
    double max = Constants.HoodConstants.kOpenHoodRot;
    setHoodRot(min + pct * (max - min));
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
    // Live-tunable gains + calibration knobs straight from Elastic (SmartDashboard).
    Constants.HoodConstants.kP = SmartDashboard.getNumber("Hood/kP", Constants.HoodConstants.kP);
    Constants.HoodConstants.kI = SmartDashboard.getNumber("Hood/kI", Constants.HoodConstants.kI);
    Constants.HoodConstants.kD = SmartDashboard.getNumber("Hood/kD", Constants.HoodConstants.kD);
    Constants.HoodConstants.kMaxOut = SmartDashboard.getNumber("Hood/MaxOut", Constants.HoodConstants.kMaxOut);
    Constants.HoodConstants.kOpenHoodRot =
        SmartDashboard.getNumber("Hood/OpenLimitRot", Constants.HoodConstants.kOpenHoodRot);
    pid.setPID(Constants.HoodConstants.kP, Constants.HoodConstants.kI, Constants.HoodConstants.kD);

    // Software-zero button (captures current reading at the closed stop, then self-resets).
    if (SmartDashboard.getBoolean("Hood/ZeroNow", false)) {
      zeroAtCurrent();
      SmartDashboard.putBoolean("Hood/ZeroNow", false);
    }

    // Dashboard angle command as a PERCENT of the working range [0, OpenLimit].
    if (SmartDashboard.getBoolean("Hood/GoToTune", false)) {
      setHoodPercentOfRange(SmartDashboard.getNumber("Hood/TunePercent", 0.0));
    }

    SmartDashboard.putNumber("Hood/HoodRot", getHoodRot());
    SmartDashboard.putNumber("Hood/PercentOpen", getHoodPercent());
    SmartDashboard.putNumber("Hood/RawEncoder", getRawEncoder());
    SmartDashboard.putNumber("Hood/TargetRot", targetHoodRot);
    SmartDashboard.putBoolean("Hood/AtAngle", isAtAngle());
    SmartDashboard.putBoolean("Hood/Zeroed", zeroed);

    double maxOut = Constants.HoodConstants.kMaxOut;
    double output;
    if (manualPercent != 0.0) {
      // Manual jog (LT/RT): open-loop, NOT limit-guarded, so you can always reach the stops (e.g.
      // to find closed and zero). Strength = Hood/JogPercent, separate from the PID's MaxOut.
      double jogCap = SmartDashboard.getNumber("Hood/JogPercent", 0.2);
      output = MathUtil.clamp(manualPercent, -jogCap, jogCap);
      targetHoodRot = clampToLimits(getHoodRot()); // hold here when released
    } else if (zeroed) {
      output = MathUtil.clamp(pid.calculate(getHoodRot(), targetHoodRot), -maxOut, maxOut);
      output = applyLimitGuard(output); // guard the PID only
    } else {
      output = 0.0; // not zeroed: only manual jog allowed (drive to closed, then Hood/ZeroNow)
    }

    output = applySlew(output);
    SmartDashboard.putNumber("Hood/CmdOutput", output);          // what we send the Spark MAX (debug)
    SmartDashboard.putNumber("Hood/ManualPercent", manualPercent); // -1 LT / 0 / +1 RT (debug)
    motor.set(output);
  }
}
