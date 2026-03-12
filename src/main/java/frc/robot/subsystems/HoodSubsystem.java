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
 * Hood:
 * - Motor 1: NEO Vortex on Spark Flex (percent output control)
 * - Motor 2: NEO 1.1 on Spark Max (hood angle, brake mode)
 * - Sensor: REV Absolute Encoder on DIO (duty cycle), single-turn [0,1)
 * - Gear ratio 1:2 (encoder rotates twice per one hood rotation)
 */
public class HoodSubsystem extends SubsystemBase {

  private final SparkFlex motor =
      new SparkFlex(Constants.HoodConstants.kHoodMotorCanId, SparkLowLevel.MotorType.kBrushless);

  private final SparkMax angleMotor =
      new SparkMax(Constants.CanId.kHoodAngleNeo, SparkLowLevel.MotorType.kBrushless);

  private final DutyCycleEncoder absEncoder =
      new DutyCycleEncoder(Constants.HoodConstants.kThroughBoreDio);

  private final PIDController pid =
      new PIDController(Constants.HoodConstants.kP, Constants.HoodConstants.kI, Constants.HoodConstants.kD);

  // Target in HOOD rotations
  private double targetHoodRot = Constants.HoodConstants.kMinHoodRot;

  // Tunable offset — set this so the encoder reads 0 when fully closed (steepest angle)
  private double encoderOffset = Constants.HoodConstants.kEncoderOffsetRot;

  public HoodSubsystem() {
    SparkFlexConfig cfg = new SparkFlexConfig();
    cfg.idleMode(SparkBaseConfig.IdleMode.kBrake);
    cfg.inverted(Constants.MotorInverts.kHoodInverted);
    motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // NEO 1.1 for hood angle — brake mode, brushless
    SparkMaxConfig angleCfg = new SparkMaxConfig();
    angleCfg.idleMode(SparkBaseConfig.IdleMode.kBrake);
    angleMotor.configure(angleCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pid.setTolerance(Constants.HoodConstants.kToleranceHoodRot);

    // Publish initial offset so it can be tuned from Elastic
    SmartDashboard.putNumber("Hood/EncoderOffset", encoderOffset);
  }

  /** Raw absolute encoder value [0, 1). */
  public double getAbsoluteEncoder() {
    return absEncoder.get();
  }

  /** Absolute encoder with offset applied (zeroed when fully closed). */
  public double getAbsoluteEncoderOffseted() {
    return getAbsoluteEncoder() - encoderOffset;
  }

  /** Hood rotation = offsetted encoder / 2 (1:2 gear ratio). */
  public double getHoodRotation() {
    return getAbsoluteEncoderOffseted() / 2.0;
  }

  /** Hood angle in degrees = rotation * 360. */
  public double getHoodAngle() {
    return getHoodRotation() * 360.0;
  }

  /** Alias used by PID and existing commands. */
  public double getHoodRot() {
    return getHoodRotation();
  }

  public double getTargetHoodRot() {
    return targetHoodRot;
  }

  /** Set target in HOOD rotations. */
  public void setHoodRot(double hoodRot) {
    targetHoodRot = AngleMath.clamp(
        hoodRot,
        Constants.HoodConstants.kMinHoodRot,
        Constants.HoodConstants.kMaxHoodRot
    );
  }

  public boolean atTarget() {
    return pid.atSetpoint();
  }

  public void stop() {
    motor.set(0.0);
  }

  @Override
  public void periodic() {
    // Read tunable offset from Elastic
    encoderOffset = SmartDashboard.getNumber("Hood/EncoderOffset", encoderOffset);

    double pos = getHoodRot();

    double out = pid.calculate(pos, targetHoodRot);
    out = AngleMath.clamp(out, -Constants.HoodConstants.kMaxOut, Constants.HoodConstants.kMaxOut);
    motor.set(out);

    // Publish all hood telemetry to Elastic
    SmartDashboard.putNumber("Hood/HoodAbsoluteEncoder", getAbsoluteEncoder());
    SmartDashboard.putNumber("Hood/HoodAbsoluteEncoderOffseted", getAbsoluteEncoderOffseted());
    SmartDashboard.putNumber("Hood/HoodRotation", getHoodRotation());
    SmartDashboard.putNumber("Hood/HoodAngle", getHoodAngle());
    SmartDashboard.putNumber("Hood/TargetRot", targetHoodRot);
    SmartDashboard.putBoolean("Hood/AtTarget", atTarget());
  }
}