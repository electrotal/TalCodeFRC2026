package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.AngleMath;

/**
 * Hood subsystem.
 * Motor: NEO Vortex on Spark Flex (CAN 60), angle motor: NEO 1.1 on Spark Max (CAN 26).
 * Sensor: REV Through-Bore encoder on DIO 3 (duty cycle, 0-1 range).
 * Gear ratio 1:2 — encoder rotates twice per one hood rotation.
 *
 * Position scale: 0.0 = fully closed, 1.0 = fully open (percent via setHoodPercent).
 */
public class HoodSubsystem extends SubsystemBase {

  // ── Hardware ────────────────────────────────────────────────────────────────

  private final SparkFlex motor =
      new SparkFlex(Constants.HoodConstants.kHoodMotorCanId, SparkLowLevel.MotorType.kBrushless);

  private final SparkMax angleMotor =
      new SparkMax(Constants.CanId.kHoodAngleNeo, SparkLowLevel.MotorType.kBrushless);

  private final DutyCycleEncoder encoder =
      new DutyCycleEncoder(Constants.HoodConstants.kThroughBoreDio);

  // ── Control ─────────────────────────────────────────────────────────────────

  private final PIDController pid =
      new PIDController(Constants.HoodConstants.kP, Constants.HoodConstants.kI, Constants.HoodConstants.kD);

  // ── Tunable state (read back from Elastic every loop) ───────────────────────

  private double encoderOffset = Constants.HoodConstants.kEncoderOffsetRot;
  private double maxOut        = Constants.HoodConstants.kMaxOut;

  // ── Setpoint ─────────────────────────────────────────────────────────────────

  private double targetHoodRot = Constants.HoodConstants.kMinHoodRot;

  // ─────────────────────────────────────────────────────────────────────────────

  public HoodSubsystem() {
    SparkFlexConfig cfg = new SparkFlexConfig();
    cfg.idleMode(SparkBaseConfig.IdleMode.kBrake);
    cfg.inverted(Constants.MotorInverts.kHoodInverted);
    motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig angleCfg = new SparkMaxConfig();
    angleCfg.idleMode(SparkBaseConfig.IdleMode.kBrake);
    angleMotor.configure(angleCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pid.setTolerance(Constants.HoodConstants.kToleranceHoodRot);

    // Publish tunable values so they show up immediately in Elastic
    SmartDashboard.putNumber("Hood/EncoderOffset", encoderOffset);
    SmartDashboard.putNumber("Hood/MaxOut",         maxOut);
    SmartDashboard.putNumber("Hood/kP",             Constants.HoodConstants.kP);
    SmartDashboard.putNumber("Hood/kI",             Constants.HoodConstants.kI);
    SmartDashboard.putNumber("Hood/kD",             Constants.HoodConstants.kD);
  }

  // ── Getters ──────────────────────────────────────────────────────────────────

  /** Raw encoder value [0, 1). */
  public double getRawEncoder() {
    return encoder.get();
  }

  /** Encoder value with offset applied. */
  public double getOffsetEncoder() {
    return getRawEncoder() - encoderOffset;
  }

  /** Hood position in rotations (gear ratio 1:2). */
  public double getHoodRot() {
    return getOffsetEncoder() / 2.0;
  }

  /** Hood position as a percentage: 0 = fully closed, 100 = fully open. */
  public double getHoodPercent() {
    double range = Constants.HoodConstants.kMaxHoodRot - Constants.HoodConstants.kMinHoodRot;
    if (range == 0) return 0.0;
    return ((getHoodRot() - Constants.HoodConstants.kMinHoodRot) / range) * 100.0;
  }

  /** True if the through-bore encoder has a valid signal. */
  public boolean isEncoderConnected() {
    return encoder.isConnected();
  }

  public boolean atTarget() {
    return pid.atSetpoint();
  }

  // ── Setters ──────────────────────────────────────────────────────────────────

  /** Set target in hood rotations. Clamped to closed/open range. */
  public void setHoodRot(double hoodRot) {
    targetHoodRot = AngleMath.clamp(
        hoodRot,
        Constants.HoodConstants.kMinHoodRot,
        Constants.HoodConstants.kMaxHoodRot);
  }

  /**
   * Set target as a percentage (0 = fully closed, 100 = fully open).
   * Maps linearly across kClosedHoodRot → kOpenHoodRot.
   */
  public void setHoodPercent(double percent) {
    double clamped = AngleMath.clamp(percent, 0.0, 100.0);
    double range   = Constants.HoodConstants.kMaxHoodRot - Constants.HoodConstants.kMinHoodRot;
    setHoodRot(Constants.HoodConstants.kMinHoodRot + (clamped / 100.0) * range);
  }

  public void stop() {
    motor.set(0.0);
  }

  // ── Periodic ─────────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    // Read tunable values from Elastic
    encoderOffset = SmartDashboard.getNumber("Hood/EncoderOffset", encoderOffset);
    maxOut        = SmartDashboard.getNumber("Hood/MaxOut",         maxOut);

    double newP = SmartDashboard.getNumber("Hood/kP", Constants.HoodConstants.kP);
    double newI = SmartDashboard.getNumber("Hood/kI", Constants.HoodConstants.kI);
    double newD = SmartDashboard.getNumber("Hood/kD", Constants.HoodConstants.kD);
    if (newP != pid.getP() || newI != pid.getI() || newD != pid.getD()) {
      pid.setPID(newP, newI, newD);
    }

    // Run PID
    double out = pid.calculate(getHoodRot(), targetHoodRot);
    out = AngleMath.clamp(out, -maxOut, maxOut);
    motor.set(out);

    // Telemetry
    SmartDashboard.putNumber("Hood/PercentOpen",      getHoodPercent());
    SmartDashboard.putBoolean("Hood/EncoderConnected", isEncoderConnected());
    SmartDashboard.putNumber("Hood/RawEncoder",        getRawEncoder());
    SmartDashboard.putNumber("Hood/OffsetEncoder",     getOffsetEncoder());
    SmartDashboard.putNumber("Hood/RotationRot",       getHoodRot());
    SmartDashboard.putNumber("Hood/TargetRot",         targetHoodRot);
    SmartDashboard.putBoolean("Hood/AtTarget",         atTarget());
  }
}
