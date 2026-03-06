package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.AngleMath;

public class ShooterSubsystem extends SubsystemBase {

  public enum ShooterMotor { TOP, MID, BOTTOM }

  private final TalonFX top = new TalonFX(Constants.CanId.kShooterTopKraken);
  private final TalonFX mid = new TalonFX(Constants.CanId.kShooterMidKraken);
  private final TalonFX bottom = new TalonFX(Constants.CanId.kShooterBottomKraken);

  private final VelocityVoltage velReq = new VelocityVoltage(0.0);
  private final DutyCycleOut pctReq = new DutyCycleOut(0.0);

  private double targetTopRpm = 0.0;
  private double targetMidRpm = 0.0;
  private double targetBottomRpm = 0.0;

  private double atSpeedSinceSec = -1.0;

  public ShooterSubsystem() {
    configureMotor(top, Constants.MotorInverts.kShooterTopInverted);
    configureMotor(mid, Constants.MotorInverts.kShooterMidInverted);
    configureMotor(bottom, Constants.MotorInverts.kShooterBottomInverted);
  }

  private void configureMotor(TalonFX motor, boolean invert) {
    // Shooter wheels should coast.
    MotorOutputConfigs out = new MotorOutputConfigs();
    out.NeutralMode = NeutralModeValue.Coast;
    out.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    motor.getConfigurator().apply(out);

    CurrentLimitsConfigs cl = new CurrentLimitsConfigs();
    cl.SupplyCurrentLimitEnable = true;
    cl.SupplyCurrentLimit = 80;
    cl.StatorCurrentLimitEnable = true;
    cl.StatorCurrentLimit = 120;
    motor.getConfigurator().apply(cl);

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = Constants.ShooterConstants.kVelocityP;
    slot0.kI = Constants.ShooterConstants.kVelocityI;
    slot0.kD = Constants.ShooterConstants.kVelocityD;
    motor.getConfigurator().apply(slot0);
  }

  public void setTargetRpm(ShooterMotor which, double rpm) {
    rpm = Math.max(0.0, rpm);
    switch (which) {
      case TOP -> targetTopRpm = rpm;
      case MID -> targetMidRpm = rpm;
      case BOTTOM -> targetBottomRpm = rpm;
    }
    applyVelocityTargets();
  }

  public void setAllTargetRpm(double rpm) {
    setTargetRpm(ShooterMotor.TOP, rpm);
    setTargetRpm(ShooterMotor.MID, rpm);
    setTargetRpm(ShooterMotor.BOTTOM, rpm);
  }

  private void applyVelocityTargets() {
    // Phoenix 6 velocity uses rps.
    top.setControl(velReq.withVelocity(targetTopRpm / 60.0));
    mid.setControl(velReq.withVelocity(targetMidRpm / 60.0));
    bottom.setControl(velReq.withVelocity(targetBottomRpm / 60.0));
  }

  public void setAllPercent(double percent) {
    percent = AngleMath.clamp(percent, -1.0, 1.0);
    top.setControl(pctReq.withOutput(percent));
    mid.setControl(pctReq.withOutput(percent));
    bottom.setControl(pctReq.withOutput(percent));
  }

  public double getTopRpm() { return top.getVelocity().getValueAsDouble() * 60.0; }
  public double getMidRpm() { return mid.getVelocity().getValueAsDouble() * 60.0; }
  public double getBottomRpm() { return bottom.getVelocity().getValueAsDouble() * 60.0; }

  public boolean isAtSpeed() {
    double tol = Constants.ShooterConstants.kReadyRpmTolerance;
    boolean topOk = Math.abs(getTopRpm() - targetTopRpm) <= tol;
    boolean midOk = Math.abs(getMidRpm() - targetMidRpm) <= tol;
    boolean botOk = Math.abs(getBottomRpm() - targetBottomRpm) <= tol;
    return topOk && midOk && botOk;
  }

  public boolean isAtSpeedForTime(double seconds) {
    double now = Timer.getFPGATimestamp();
    if (isAtSpeed()) {
      if (atSpeedSinceSec < 0.0) atSpeedSinceSec = now;
      return (now - atSpeedSinceSec) >= seconds;
    }
    atSpeedSinceSec = -1.0;
    return false;
  }

  public void stop() {
    setAllTargetRpm(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Shooter/Ready", isAtSpeed());
    SmartDashboard.putNumber("Shooter/TopRPM", getTopRpm());
    SmartDashboard.putNumber("Shooter/MidRPM", getMidRpm());
    SmartDashboard.putNumber("Shooter/BottomRPM", getBottomRpm());
    SmartDashboard.putBoolean("Shooter/Ready", isAtSpeed());
  }
}   