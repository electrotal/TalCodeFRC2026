package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.AngleMath;
import frc.robot.util.MultiTurnAbsoluteEncoder;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX pivotMotor = new TalonFX(Constants.CanId.kIntakePivotKraken);
  private final TalonFX rollerMotor = new TalonFX(Constants.CanId.kIntakeRollerKraken);

  // Through bore absolute encoder on the small sprocket
  private final MultiTurnAbsoluteEncoder throughBore =
      new MultiTurnAbsoluteEncoder(Constants.IntakeConstants.kThroughBoreDutyCycleDio);

  private final PIDController pivotPid =
      new PIDController(
          Constants.IntakeConstants.kPivotP,
          Constants.IntakeConstants.kPivotI,
          Constants.IntakeConstants.kPivotD);

  // All position units are encoder rotations after zero offset is applied.
  private double openRot = Constants.IntakeConstants.kOpenPivotRot;
  private double closedRot = Constants.IntakeConstants.kClosedPivotRot;
  private double targetRot = closedRot;

  private boolean open = false;

  private double rollerPercent = Constants.IntakeConstants.kRollerPercent;
  private boolean pivotHoldEnabled = true;

  private double maxOut = Constants.IntakeConstants.kPivotMaxOut;
  private double toleranceRot = Constants.IntakeConstants.kPivotToleranceRot;

  private final DutyCycleOut pivotOut = new DutyCycleOut(0.0);
  private final DutyCycleOut rollerOut = new DutyCycleOut(0.0);

  private double lastPivotPercent = 0.0;
  private double lastRollerPercent = 0.0;

  public IntakeSubsystem() {
    configureMotor(pivotMotor, NeutralModeValue.Brake, Constants.MotorInverts.kIntakePivotInverted);
    configureMotor(rollerMotor, NeutralModeValue.Coast, Constants.MotorInverts.kIntakeRollerInverted);

    pivotPid.setTolerance(toleranceRot);

    SmartDashboard.putNumber("Intake/P", pivotPid.getP());
    SmartDashboard.putNumber("Intake/I", pivotPid.getI());
    SmartDashboard.putNumber("Intake/D", pivotPid.getD());

    SmartDashboard.putNumber("Intake/OpenRot", openRot);
    SmartDashboard.putNumber("Intake/ClosedRot", closedRot);
    SmartDashboard.putNumber("Intake/TargetRot", targetRot);
    SmartDashboard.putNumber("Intake/RollerPercent", rollerPercent);
    SmartDashboard.putNumber("Intake/MaxOut", maxOut);
    SmartDashboard.putNumber("Intake/ToleranceRot", toleranceRot);
    SmartDashboard.putBoolean("Intake/PivotHoldEnabled", pivotHoldEnabled);
    SmartDashboard.putNumber("Intake/ZeroOffsetRot", Constants.IntakeConstants.kPivotEncoderZeroOffsetRot);
  }

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

  private static double wrap01(double x) {
    double r = x % 1.0;
    if (r < 0) r += 1.0;
    return r;
  }

  private static double wrapHalf(double x) {
    return wrap01(x + 0.5) - 0.5;
  }

  public double getAbs01() {
    return wrap01(throughBore.getAbsRot());
  }

  public double getRel01() {
    return wrap01(getAbs01() - Constants.IntakeConstants.kPivotEncoderZeroOffsetRot);
  }

  public boolean isOpen() {
    return open;
  }

  public boolean isPivotHoldEnabled() {
    return pivotHoldEnabled;
  }

  public void setPivotHoldEnabled(boolean enabled) {
    pivotHoldEnabled = enabled;
    SmartDashboard.putBoolean("Intake/PivotHoldEnabled", pivotHoldEnabled);
    if (!enabled) {
      lastPivotPercent = 0.0;
      pivotMotor.setControl(pivotOut.withOutput(0.0));
    }
  }

  public void togglePivotHoldEnabled() {
    setPivotHoldEnabled(!pivotHoldEnabled);
  }

  public void setPivotTargetRot(double rot) {
    targetRot = wrap01(rot);
    SmartDashboard.putNumber("Intake/TargetRot", targetRot);
  }

  public double getPivotTargetRot() {
    return targetRot;
  }

  public double getThroughBoreContinuousRot() {
    return throughBore.getContinuousRot();
  }

  public double getMeasuredRot() {
    return getRel01();
  }

  public double getPivotRot() {
    return getMeasuredRot();
  }

  public double getPivotErrorRot() {
    return wrapHalf(targetRot - getMeasuredRot());
  }

  public double getLastPivotPercent() {
    return lastPivotPercent;
  }

  public double getLastRollerPercent() {
    return lastRollerPercent;
  }

  public double getPivotMotorRotorPosRot() {
    return pivotMotor.getPosition().getValueAsDouble();
  }

  public double getPivotMotorRotorVelRps() {
    return pivotMotor.getVelocity().getValueAsDouble();
  }

  public double getRollerMotorRotorVelRps() {
    return rollerMotor.getVelocity().getValueAsDouble();
  }

  public void open() {
    open = true;
    setPivotTargetRot(openRot);
    setRollerPercent(rollerPercent);
  }

  public void close() {
    open = false;
    stopRoller();
    setPivotTargetRot(closedRot);
  }

  public void toggle() {
    if (open) {
      close();
    } else {
      open();
    }
  }

  public void setRollerPercent(double percent) {
    percent = AngleMath.clamp(percent, -1.0, 1.0);
    lastRollerPercent = percent;
    rollerMotor.setControl(rollerOut.withOutput(percent));
  }

  public void stopRoller() {
    setRollerPercent(0.0);
  }

  private void updateTunablesFromNT() {
    double p = SmartDashboard.getNumber("Intake/P", pivotPid.getP());
    double i = SmartDashboard.getNumber("Intake/I", pivotPid.getI());
    double d = SmartDashboard.getNumber("Intake/D", pivotPid.getD());
    pivotPid.setPID(p, i, d);

    openRot = wrap01(SmartDashboard.getNumber("Intake/OpenRot", openRot));
    closedRot = wrap01(SmartDashboard.getNumber("Intake/ClosedRot", closedRot));
    targetRot = wrap01(SmartDashboard.getNumber("Intake/TargetRot", targetRot));

    rollerPercent = SmartDashboard.getNumber("Intake/RollerPercent", rollerPercent);
    maxOut = SmartDashboard.getNumber("Intake/MaxOut", maxOut);
    toleranceRot = SmartDashboard.getNumber("Intake/ToleranceRot", toleranceRot);
    pivotPid.setTolerance(toleranceRot);

    pivotHoldEnabled = SmartDashboard.getBoolean("Intake/PivotHoldEnabled", pivotHoldEnabled);
  }

  @Override
  public void periodic() {
    updateTunablesFromNT();

    SmartDashboard.putString("Intake/State", open ? "Open" : "Closed");
    SmartDashboard.putBoolean("Intake/EncoderConnected", throughBore.isConnected());
    SmartDashboard.putNumber("Intake/Abs01", getAbs01());
    SmartDashboard.putNumber("Intake/Rel01", getRel01());
    SmartDashboard.putNumber("Intake/MeasRot", getMeasuredRot());
    SmartDashboard.putNumber("Intake/ErrRot", getPivotErrorRot());
    SmartDashboard.putNumber("Intake/ThroughBoreContRot", getThroughBoreContinuousRot());
    SmartDashboard.putNumber("Intake/PivotCmdPct", lastPivotPercent);
    SmartDashboard.putNumber("Intake/RollerCmdPct", lastRollerPercent);
    SmartDashboard.putNumber("Intake/PivotMotorRotorPosRot", getPivotMotorRotorPosRot());
    SmartDashboard.putNumber("Intake/PivotMotorRotorVelRps", getPivotMotorRotorVelRps());
    SmartDashboard.putNumber("Intake/RollerMotorRotorVelRps", getRollerMotorRotorVelRps());
    SmartDashboard.putBoolean("Intake/Open", open);
    SmartDashboard.putBoolean("Intake/PivotHoldEnabled", pivotHoldEnabled);

    if (!pivotHoldEnabled) {
      lastPivotPercent = 0.0;
      pivotMotor.setControl(pivotOut.withOutput(0.0));
      return;
    }

    double out = pivotPid.calculate(getMeasuredRot(), targetRot);

    out = AngleMath.clamp(out, -maxOut, maxOut);
    lastPivotPercent = out;
    pivotMotor.setControl(pivotOut.withOutput(out));
  }
}
