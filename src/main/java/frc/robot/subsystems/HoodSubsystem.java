package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
//import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.AngleMath;
import frc.robot.util.MultiTurnAbsoluteEncoder;

/**
 * Hood:
 * - Motor: NEO Vortex on Spark Flex (percent output control)
 * - Sensor: REV Through Bore encoder on DIO (duty cycle), tracked as multi-turn
 *
 * We keep the encoder in absolute mode, but we count wraps in software to get continuous rotations.
 */
public class HoodSubsystem extends SubsystemBase {

  private final SparkFlex motor =
      new SparkFlex(Constants.HoodConstants.kHoodMotorCanId, SparkLowLevel.MotorType.kBrushless);

  // Reuse the same multi-turn wrapper you already use for intake
  private final MultiTurnAbsoluteEncoder encoder =
      new MultiTurnAbsoluteEncoder(Constants.HoodConstants.kThroughBoreDio);

  private final PIDController pid =
      new PIDController(Constants.HoodConstants.kP, Constants.HoodConstants.kI, Constants.HoodConstants.kD);

  // Target in HOOD rotations
  private double targetHoodRot = Constants.HoodConstants.kMinHoodRot;

  public HoodSubsystem() {
    // REVLib 2026 config API
    SparkFlexConfig cfg = new SparkFlexConfig();
    cfg.idleMode(SparkBaseConfig.IdleMode.kBrake);
    cfg.inverted(Constants.MotorInverts.kHoodInverted);

    // Use the non-deprecated configure signature (ResetMode/PersistMode from com.revrobotics)
    motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pid.setTolerance(Constants.HoodConstants.kToleranceHoodRot);
  }

  /**
   * Hood position in HOOD rotations.
   * encoder.getContinuousRot() returns encoder rotations (can exceed 1.0).
   * We apply zero offset and gear ratio to convert to hood rotations.
   */
  public double getHoodRot() {
    double encRot = encoder.getContinuousRot() - Constants.HoodConstants.kEncoderOffsetRot;
    return encRot * Constants.HoodConstants.kEncoderToHoodRatio;
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
    double pos = getHoodRot();

    double out = pid.calculate(pos, targetHoodRot);
    out = AngleMath.clamp(out, -Constants.HoodConstants.kMaxOut, Constants.HoodConstants.kMaxOut);

    motor.set(out);

    SmartDashboard.putNumber("Hood/PosRot", pos);
    SmartDashboard.putNumber("Hood/TargetRot", targetHoodRot);
    SmartDashboard.putBoolean("Hood/AtTarget", atTarget());
  }
}