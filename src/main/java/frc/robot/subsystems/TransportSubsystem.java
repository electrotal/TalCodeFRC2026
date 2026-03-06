package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.AngleMath;

public class TransportSubsystem extends SubsystemBase {

  private final TalonFX mainConveyor = new TalonFX(Constants.CanId.kMainTransportKraken);
  private final TalonFX shooterFeeder = new TalonFX(Constants.CanId.kShooterFeederKraken);

  private final DutyCycleOut mainOut = new DutyCycleOut(0.0);
  private final DutyCycleOut feederOut = new DutyCycleOut(0.0);

  private double lastMainPercent = 0.0;
  private double lastFeederPercent = 0.0;

  private double liveTransportPercent = Constants.TransportConstants.kTransportPercent;
  private double liveFeederPercent    = Constants.TransportConstants.kFeederOnlyPercent;

  public TransportSubsystem() {
    // Transport should coast.
    configureMotor(mainConveyor, NeutralModeValue.Coast, Constants.MotorInverts.kMainTransportInverted);
    configureMotor(shooterFeeder, NeutralModeValue.Coast, Constants.MotorInverts.kShooterFeederInverted);

    SmartDashboard.putNumber("Transport/TransportPercent", liveTransportPercent);
    SmartDashboard.putNumber("Transport/FeederPercent",    liveFeederPercent);
  }

  public double getTransportPercent() { return liveTransportPercent; }
  public double getFeederPercent()    { return liveFeederPercent; }

  private static void configureMotor(TalonFX motor, NeutralModeValue neutralMode, boolean invert) {
    MotorOutputConfigs out = new MotorOutputConfigs();
    out.NeutralMode = neutralMode;
    out.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    motor.getConfigurator().apply(out);

    CurrentLimitsConfigs cl = new CurrentLimitsConfigs();
    cl.SupplyCurrentLimitEnable = true;
    cl.SupplyCurrentLimit = 60;
    cl.StatorCurrentLimitEnable = true;
    cl.StatorCurrentLimit = 60;
    motor.getConfigurator().apply(cl);
  }

  public double getLastMainPercent() {
    return lastMainPercent;
  }

  public double getLastFeederPercent() {
    return lastFeederPercent;
  }

  public double getMainRotorPosRot() {
    return mainConveyor.getPosition().getValueAsDouble();
  }

  public double getMainRotorVelRps() {
    return mainConveyor.getVelocity().getValueAsDouble();
  }

  public double getFeederRotorPosRot() {
    return shooterFeeder.getPosition().getValueAsDouble();
  }

  public double getFeederRotorVelRps() {
    return shooterFeeder.getVelocity().getValueAsDouble();
  }

  // Main conveyor percent output.
  public void setMainConveyorPercent(double percent) {
    percent = AngleMath.clamp(percent, -1.0, 1.0);
    lastMainPercent = percent;
    mainConveyor.setControl(mainOut.withOutput(percent));
  }

  // Feeder percent output.
  public void setShooterFeederPercent(double percent) {
    percent = AngleMath.clamp(percent, -1.0, 1.0);
    lastFeederPercent = percent;
    shooterFeeder.setControl(feederOut.withOutput(percent));
  }

  public void runAll(double percent) {
    setMainConveyorPercent(percent);
    setShooterFeederPercent(percent);
  }

  public void runFeederOnly(double percent) {
    setMainConveyorPercent(0.0);
    setShooterFeederPercent(percent);
  }

  public void runConveyorOnly(double percent) {
    setMainConveyorPercent(percent);
    setShooterFeederPercent(0.0);
  }

  public void stopAll() {
    runAll(0.0);
  }

  @Override
  public void periodic() {
    liveTransportPercent = SmartDashboard.getNumber("Transport/TransportPercent", liveTransportPercent);
    liveFeederPercent    = SmartDashboard.getNumber("Transport/FeederPercent",    liveFeederPercent);

    // Diagnostics — use these in Elastic to debug the motor
    SmartDashboard.putNumber("Transport/MainCmdPct",    lastMainPercent);
    SmartDashboard.putNumber("Transport/FeederCmdPct",  lastFeederPercent);
    SmartDashboard.putNumber("Transport/MainVelRps",    getMainRotorVelRps());
    SmartDashboard.putNumber("Transport/FeederVelRps",  getFeederRotorVelRps());
    SmartDashboard.putNumber("Transport/MainStatorA",   mainConveyor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Transport/FeederStatorA", shooterFeeder.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Transport/MainSupplyV",   mainConveyor.getSupplyVoltage().getValueAsDouble());
  }
}