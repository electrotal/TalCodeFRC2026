package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.util.AngleMath;
import frc.robot.util.MultiTurnAbsoluteEncoder;

/**
 * Hood subsystem.
 * Motor: NEO 1.1 on Spark Max (CAN 26).
 * Sensor: REV Through-Bore encoder on DIO 3 (absolute, 0-1 range).
 * Gear ratio 1:2 — encoder rotates twice per one hood rotation.
 *
 * Position scale: 0% = fully closed, 100% = fully open.
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

    SmartDashboard.putNumber("Hood/EncoderOffset", encoderOffset);
  }

  // ── Getters ──────────────────────────────────────────────────────────────────

  /** Raw continuous encoder value (multi-turn, can exceed 1.0 or be negative). */
  public double getRawEncoder() {
    return encoder.getPosition();
    }

  /** Continuous encoder value with offset applied. */
  public double getOffsetEncoder() {
    return getRawEncoder() - encoderOffset;
  }

  /** Hood position in rotations (gear ratio 1:2 — encoder does 2 turns per 1 hood turn). */
  public double getHoodRot() {
    return getOffsetEncoder() / 2.0;
  }

  /** Hood position as a percentage: 0 = fully closed, 100 = fully open. */
  public double getHoodPercent() {
    double range = Constants.HoodConstants.kMaxHoodRot - Constants.HoodConstants.kMinHoodRot;
    if (range == 0) return 0.0;
    return ((getHoodRot() - Constants.HoodConstants.kMinHoodRot) / range) * 100.0;
  }


  // ── Setters ──────────────────────────────────────────────────────────────────

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

  // ── Periodic ─────────────────────────────────────────────────────────────────

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