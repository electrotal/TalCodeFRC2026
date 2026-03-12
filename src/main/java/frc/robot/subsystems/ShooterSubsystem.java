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

  private final VelocityVoltage topVelReq = new VelocityVoltage(0.0).withSlot(0);
  private final VelocityVoltage midVelReq = new VelocityVoltage(0.0).withSlot(0);
  private final VelocityVoltage bottomVelReq = new VelocityVoltage(0.0).withSlot(0);
  private final DutyCycleOut pctReq = new DutyCycleOut(0.0);

  private double targetTopRpm = 0.0;
  private double targetMidRpm = 0.0;
  private double targetBottomRpm = 0.0;

  private boolean velocityControlEnabled = false;
  private double atSpeedSinceSec = -1.0;

  // Live-tunable toggle RPMs (editable from SmartDashboard / Elastic)
  private double liveLowRpm  = Constants.ShooterConstants.kToggleTestLowRpm;
  private double liveHighRpm = Constants.ShooterConstants.kToggleTestHighRpm;

  // Per-motor kill switches — toggle from Elastic to disable individual shooters
  private boolean rightShooterOn  = true;
  private boolean middleShooterOn = true;
  private boolean leftShooterOn   = true;

  // Live-tunable PID gains
  private double liveKP = Constants.ShooterConstants.kVelocityP;
  private double liveKI = Constants.ShooterConstants.kVelocityI;
  private double liveKD = Constants.ShooterConstants.kVelocityD;
  private double liveKV = Constants.ShooterConstants.kVelocityV;
  private double liveKS = Constants.ShooterConstants.kVelocityS;

  public ShooterSubsystem() {
    configureMotor(top, Constants.MotorInverts.kShooterTopInverted);
    configureMotor(mid, Constants.MotorInverts.kShooterMidInverted);
    configureMotor(bottom, Constants.MotorInverts.kShooterBottomInverted);

    SmartDashboard.putNumber("Shooter/ToggleLowRpm",  liveLowRpm);
    SmartDashboard.putNumber("Shooter/ToggleHighRpm", liveHighRpm);
    SmartDashboard.putBoolean("Shooter/RightShooterOn",  rightShooterOn);
    SmartDashboard.putBoolean("Shooter/MiddleShooterOn", middleShooterOn);
    SmartDashboard.putBoolean("Shooter/LeftShooterOn",   leftShooterOn);

    SmartDashboard.putNumber("Shooter/PID/kP", liveKP);
    SmartDashboard.putNumber("Shooter/PID/kI", liveKI);
    SmartDashboard.putNumber("Shooter/PID/kD", liveKD);
    SmartDashboard.putNumber("Shooter/PID/kV", liveKV);
    SmartDashboard.putNumber("Shooter/PID/kS", liveKS);
  }

  private void configureMotor(TalonFX motor, boolean invert) {
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
    slot0.kV = Constants.ShooterConstants.kVelocityV;
    slot0.kS = Constants.ShooterConstants.kVelocityS;
    motor.getConfigurator().apply(slot0);
  }

  public void setTargetRpm(ShooterMotor which, double rpm) {
    rpm = Math.max(0.0, rpm);
    switch (which) {
      case TOP -> targetTopRpm = rpm;
      case MID -> targetMidRpm = rpm;
      case BOTTOM -> targetBottomRpm = rpm;
    }
    velocityControlEnabled = true;
    applyVelocityTargets();
  }

  public void setTargetRpms(double topRpm, double midRpm, double bottomRpm) {
    targetTopRpm = Math.max(0.0, topRpm);
    targetMidRpm = Math.max(0.0, midRpm);
    targetBottomRpm = Math.max(0.0, bottomRpm);
    velocityControlEnabled = true;
    applyVelocityTargets();
  }

  public void setAllTargetRpm(double rpm) {
    setTargetRpms(rpm, rpm, rpm);
  }

  private void applyVelocityTargets() {
    top.setControl(topVelReq.withVelocity(rightShooterOn  ? targetTopRpm / 60.0 : 0.0));
    mid.setControl(midVelReq.withVelocity(middleShooterOn ? targetMidRpm / 60.0 : 0.0));
    bottom.setControl(bottomVelReq.withVelocity(leftShooterOn ? targetBottomRpm / 60.0 : 0.0));
  }

  public void setAllPercent(double percent) {
    velocityControlEnabled = false;
    percent = AngleMath.clamp(percent, -1.0, 1.0);
    top.setControl(pctReq.withOutput(rightShooterOn  ? percent : 0.0));
    mid.setControl(pctReq.withOutput(middleShooterOn ? percent : 0.0));
    bottom.setControl(pctReq.withOutput(leftShooterOn ? percent : 0.0));
  }

  public double getTopRpm() {
    return top.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getMidRpm() {
    return mid.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getBottomRpm() {
    return bottom.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getTargetTopRpm() {
    return targetTopRpm;
  }

  public double getTargetMidRpm() {
    return targetMidRpm;
  }

  public double getTargetBottomRpm() {
    return targetBottomRpm;
  }

  public double getLiveLowRpm()  { return liveLowRpm; }
  public double getLiveHighRpm() { return liveHighRpm; }

  public boolean isAtSpeed() {
    double tol = Constants.ShooterConstants.kReadyRpmTolerance;
    boolean topOk = Math.abs(getTopRpm() - targetTopRpm) <= tol;
    boolean midOk = Math.abs(getMidRpm() - targetMidRpm) <= tol;
    boolean botOk = Math.abs(getBottomRpm() - targetBottomRpm) <= tol;
    return velocityControlEnabled && topOk && midOk && botOk;
  }

  public boolean isAtSpeedForTime(double seconds) {
    double now = Timer.getFPGATimestamp();
    if (isAtSpeed()) {
      if (atSpeedSinceSec < 0.0) {
        atSpeedSinceSec = now;
      }
      return (now - atSpeedSinceSec) >= seconds;
    }
    atSpeedSinceSec = -1.0;
    return false;
  }

  public void stop() {
    targetTopRpm = 0.0;
    targetMidRpm = 0.0;
    targetBottomRpm = 0.0;
    velocityControlEnabled = false;
    atSpeedSinceSec = -1.0;
    setAllPercent(0.0);
  }

  @Override
  public void periodic() {
    liveLowRpm  = SmartDashboard.getNumber("Shooter/ToggleLowRpm",  liveLowRpm);
    liveHighRpm = SmartDashboard.getNumber("Shooter/ToggleHighRpm", liveHighRpm);

    // Read kill switches from Elastic
    rightShooterOn  = SmartDashboard.getBoolean("Shooter/RightShooterOn",  rightShooterOn);
    middleShooterOn = SmartDashboard.getBoolean("Shooter/MiddleShooterOn", middleShooterOn);
    leftShooterOn   = SmartDashboard.getBoolean("Shooter/LeftShooterOn",   leftShooterOn);

    // Read PID gains from Elastic and hot-apply if changed
    double newKP = SmartDashboard.getNumber("Shooter/PID/kP", liveKP);
    double newKI = SmartDashboard.getNumber("Shooter/PID/kI", liveKI);
    double newKD = SmartDashboard.getNumber("Shooter/PID/kD", liveKD);
    double newKV = SmartDashboard.getNumber("Shooter/PID/kV", liveKV);
    double newKS = SmartDashboard.getNumber("Shooter/PID/kS", liveKS);

    if (newKP != liveKP || newKI != liveKI || newKD != liveKD || newKV != liveKV || newKS != liveKS) {
      liveKP = newKP; liveKI = newKI; liveKD = newKD; liveKV = newKV; liveKS = newKS;
      Slot0Configs slot0 = new Slot0Configs();
      slot0.kP = liveKP; slot0.kI = liveKI; slot0.kD = liveKD; slot0.kV = liveKV; slot0.kS = liveKS;
      top.getConfigurator().apply(slot0);
      mid.getConfigurator().apply(slot0);
      bottom.getConfigurator().apply(slot0);
    }

    if (velocityControlEnabled) {
      applyVelocityTargets();
    }

    SmartDashboard.putBoolean("Shooter/VelocityModeEnabled", velocityControlEnabled);
    SmartDashboard.putBoolean("Shooter/Ready", isAtSpeed());
    SmartDashboard.putNumber("Shooter/TopRPM", getTopRpm());
    SmartDashboard.putNumber("Shooter/MidRPM", getMidRpm());
    SmartDashboard.putNumber("Shooter/BottomRPM", getBottomRpm());
    SmartDashboard.putNumber("Shooter/TargetTopRPM", targetTopRpm);
    SmartDashboard.putNumber("Shooter/TargetMidRPM", targetMidRpm);
    SmartDashboard.putNumber("Shooter/TargetBottomRPM", targetBottomRpm);
  }
}