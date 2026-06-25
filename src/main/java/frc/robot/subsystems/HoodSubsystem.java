package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Hood subsystem — closed-loop position control.
 *
 * Motor:  NEO 1.1 on Spark Max (CAN 26).
 * Sensor: NEO relative encoder (motor.getEncoder()). The absolute through-bore
 *         was unreliable, so the hood is driven off the relative encoder; it
 *         starts at 0 every boot, so re-zero with {@link #zero()} if needed.
 *
 * Control: a roboRIO-side {@link PIDController} runs in {@link #periodic()} and
 *          drives the hood to {@code targetRot}. Position is in encoder rotations
 *          over the range [kMinPos, kMaxPos] (0 = fully closed, max = fully open).
 *          Gains, max output and (separately) a tune target are live-tunable from
 *          Elastic/SmartDashboard so the angle command can be dialed in on the fly.
 */
public class HoodSubsystem extends SubsystemBase {

  // ── Hardware ────────────────────────────────────────────────────────────────

  private final SparkMax motor =
      new SparkMax(Constants.CanId.kHoodAngleNeo, SparkLowLevel.MotorType.kBrushless);

  private final RelativeEncoder encoder;

  private final PIDController pid =
      new PIDController(
          Constants.HoodConstants.kAngleP,
          Constants.HoodConstants.kAngleI,
          Constants.HoodConstants.kAngleD);

  // ── State ─────────────────────────────────────────────────────────────────────

  private double encoderOffset = Constants.HoodConstants.kEncoderOffsetRot;
  private double maxOut        = Constants.HoodConstants.kMaxOut;
  private double targetRot     = Constants.HoodConstants.kMinPos;

  /** When true, periodic() actively drives the PID. Manual setSpeed() turns it off. */
  private boolean closedLoopEnabled = false;

  // ─────────────────────────────────────────────────────────────────────────────

  public HoodSubsystem() {
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.idleMode(SparkBaseConfig.IdleMode.kBrake);
    cfg.inverted(Constants.MotorInverts.kHoodInverted);
    cfg.smartCurrentLimit(30); // protect the NEO/gearbox if the hood stalls on a hard stop
    motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();

    pid.setTolerance(Constants.HoodConstants.kToleranceHoodRot);

    // Publish tunables so they show up immediately in Elastic.
    SmartDashboard.putNumber("Hood/kP",             Constants.HoodConstants.kAngleP);
    SmartDashboard.putNumber("Hood/kI",             Constants.HoodConstants.kAngleI);
    SmartDashboard.putNumber("Hood/kD",             Constants.HoodConstants.kAngleD);
    SmartDashboard.putNumber("Hood/MaxOut",         maxOut);
    SmartDashboard.putNumber("Hood/EncoderOffset",  encoderOffset);
    SmartDashboard.putNumber("Hood/TuneTargetRot",  targetRot);
  }

  // ── Getters ──────────────────────────────────────────────────────────────────

  /** Raw relative-encoder value in rotations. */
  public double getRawEncoder() {
    return encoder.getPosition();
  }

  /** Encoder value with the zero offset applied. */
  public double getOffsetEncoder() {
    return getRawEncoder() - encoderOffset;
  }

  /** Hood position in rotations — the PID control variable, range [kMinPos, kMaxPos]. */
  public double getHoodRot() {
    return getOffsetEncoder();
  }

  /** Hood position as a percentage: 0 = fully closed, 100 = fully open. */
  public double getHoodPercent() {
    double range = Constants.HoodConstants.kMaxPos - Constants.HoodConstants.kMinPos;
    if (range == 0) return 0.0;
    return ((getHoodRot() - Constants.HoodConstants.kMinPos) / range) * 100.0;
  }

  /** Current PID target in rotations. */
  public double getTargetHoodRot() {
    return targetRot;
  }

  /** True once the hood is within tolerance of its target. */
  public boolean atTarget() {
    return closedLoopEnabled && pid.atSetpoint();
  }

  // ── Setters ──────────────────────────────────────────────────────────────────

  /** Command a hood target in rotations. Clamped to [kMinPos, kMaxPos]; engages closed loop. */
  public void setHoodPosition(double rot) {
    targetRot = MathUtil.clamp(rot, Constants.HoodConstants.kMinPos, Constants.HoodConstants.kMaxPos);
    closedLoopEnabled = true;
  }

  /** Command a hood target as a percentage (0 = closed, 100 = open). Engages closed loop. */
  public void setHoodPercent(double percent) {
    double clamped = MathUtil.clamp(percent, 0.0, 100.0);
    double range   = Constants.HoodConstants.kMaxPos - Constants.HoodConstants.kMinPos;
    setHoodPosition(Constants.HoodConstants.kMinPos + (clamped / 100.0) * range);
  }

  /** Open-loop manual jog. Disables closed loop until a position is commanded again. */
  public void setSpeed(double speed) {
    closedLoopEnabled = false;
    motor.set(speed);
  }

  public void stop() {
    closedLoopEnabled = false;
    motor.set(0.0);
  }

  /** Re-zero: make the current physical position read as kMinPos. */
  public void zero() {
    encoderOffset = getRawEncoder() - Constants.HoodConstants.kMinPos;
    // Keep the dashboard copy in sync so periodic()'s read-back doesn't undo it.
    SmartDashboard.putNumber("Hood/EncoderOffset", encoderOffset);
  }

  // ── Periodic ─────────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    // Live tuning: pull gains, max output and zero offset back from Elastic.
    double newP = SmartDashboard.getNumber("Hood/kP", pid.getP());
    double newI = SmartDashboard.getNumber("Hood/kI", pid.getI());
    double newD = SmartDashboard.getNumber("Hood/kD", pid.getD());
    if (newP != pid.getP() || newI != pid.getI() || newD != pid.getD()) {
      pid.setPID(newP, newI, newD);
    }
    maxOut        = SmartDashboard.getNumber("Hood/MaxOut",        maxOut);
    encoderOffset = SmartDashboard.getNumber("Hood/EncoderOffset", encoderOffset);

    if (closedLoopEnabled) {
      double out = pid.calculate(getHoodRot(), targetRot);
      out = MathUtil.clamp(out, -maxOut, maxOut);
      motor.set(out);
    }

    // Telemetry
    SmartDashboard.putNumber("Hood/RawEncoder",    getRawEncoder());
    SmartDashboard.putNumber("Hood/OffsetEncoder", getOffsetEncoder());
    SmartDashboard.putNumber("Hood/HoodRot",       getHoodRot());
    SmartDashboard.putNumber("Hood/PercentOpen",   getHoodPercent());
    SmartDashboard.putNumber("Hood/TargetRot",     targetRot);
    SmartDashboard.putNumber("Hood/Error",         targetRot - getHoodRot());
    SmartDashboard.putBoolean("Hood/ClosedLoop",   closedLoopEnabled);
    SmartDashboard.putBoolean("Hood/AtTarget",     atTarget());
  }
}
