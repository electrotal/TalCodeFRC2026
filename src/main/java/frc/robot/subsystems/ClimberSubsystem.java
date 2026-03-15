package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.AngleMath;

public class ClimberSubsystem extends SubsystemBase {

  private final TalonFX climber = new TalonFX(Constants.CanId.kClimberKraken);
  private final DutyCycleOut out = new DutyCycleOut(0.0);

  private double lastPercent = 0.0;

  public ClimberSubsystem() {
    MotorOutputConfigs moc = new MotorOutputConfigs();
    moc.NeutralMode = NeutralModeValue.Brake;
    moc.Inverted = Constants.MotorInverts.kClimberInverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    climber.getConfigurator().apply(moc);

    CurrentLimitsConfigs cfg = new CurrentLimitsConfigs();
    cfg.SupplyCurrentLimitEnable = true;
    cfg.SupplyCurrentLimit = 60;
    cfg.StatorCurrentLimitEnable = true;
    cfg.StatorCurrentLimit = Constants.ClimberConstants.kStatorCurrentLimit;
    climber.getConfigurator().apply(cfg);
  }

  public void setPercent(double percent) {
    percent = AngleMath.clamp(percent, -1.0, 1.0);
    lastPercent = percent;
    climber.setControl(out.withOutput(percent));
  }

  public void stop() {
    setPercent(0.0);
  }

  public double getLastPercent() {
    return lastPercent;
  }

  public double getRotorPosRot() {
    return climber.getPosition().getValueAsDouble();
  }

  public double getRotorVelRps() {
    return climber.getVelocity().getValueAsDouble();
  }
}