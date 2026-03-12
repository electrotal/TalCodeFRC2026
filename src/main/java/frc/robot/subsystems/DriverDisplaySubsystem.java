package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FieldTargetUtil;
import swervelib.SwerveDrive;

import java.lang.reflect.Array;
import java.lang.reflect.Method;

/**
 * Publishes key values to SmartDashboard for driver and programmer bringup.
 * Uses reflection for swerve module telemetry to survive YAGSL version differences.
 */
public class DriverDisplaySubsystem extends SubsystemBase {

  private final SwerveSubsystem swerve;
  private final ShooterSubsystem shooter;
  private final HoodSubsystem hood;
  private final IntakeSubsystem intake;
  private final TransportSubsystem transport;
  private final ClimberSubsystem climber;
  private final VisionSubsystem vision;

  public DriverDisplaySubsystem(
      SwerveSubsystem swerve,
      ShooterSubsystem shooter,
      HoodSubsystem hood,
      IntakeSubsystem intake,
      TransportSubsystem transport,
      ClimberSubsystem climber,
      VisionSubsystem vision
  ) {
    this.swerve = swerve;
    this.shooter = shooter;
    this.hood = hood;
    this.intake = intake;
    this.transport = transport;
    this.climber = climber;
    this.vision = vision;
  }

  @Override
  public void periodic() {
    publishRobotHealth();
    publishDrive();
    publishShooter();
    publishHood();
    publishIntake();
    publishTransport();
    publishClimber();
    publishVisionBasics();
    publishSwerveModulesBestEffort();
  }

  private void publishRobotHealth() {
    SmartDashboard.putNumber("Robot/BatteryV", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean("Robot/BrownedOut", RobotController.isBrownedOut());

    var can = RobotController.getCANStatus();
    SmartDashboard.putNumber("Robot/CANUtil", can.percentBusUtilization);
    SmartDashboard.putNumber("Robot/CANBusOff", can.busOffCount);
    SmartDashboard.putNumber("Robot/CANTxFull", can.txFullCount);
    SmartDashboard.putNumber("Robot/CANRxErr", can.receiveErrorCount);
    SmartDashboard.putNumber("Robot/CANTxErr", can.transmitErrorCount);

    SmartDashboard.putBoolean("DS/Enabled", DriverStation.isEnabled());
    SmartDashboard.putBoolean("DS/Autonomous", DriverStation.isAutonomous());
    SmartDashboard.putBoolean("DS/Teleop", DriverStation.isTeleop());
    SmartDashboard.putBoolean("DS/Estopped", DriverStation.isEStopped());
  }

  private void publishDrive() {
    SmartDashboard.putNumber("Drive/PoseX", swerve.getPose().getX());
    SmartDashboard.putNumber("Drive/PoseY", swerve.getPose().getY());
    SmartDashboard.putNumber("Drive/HeadingDeg", swerve.getHeading().getDegrees());

    SmartDashboard.putNumber("Drive/DistToHubM", FieldTargetUtil.distanceToHubMeters(swerve.getPose()));

    ChassisSpeeds rr = swerve.getRobotRelativeSpeeds();
    SmartDashboard.putNumber("Drive/RR_vx_mps", rr.vxMetersPerSecond);
    SmartDashboard.putNumber("Drive/RR_vy_mps", rr.vyMetersPerSecond);
    SmartDashboard.putNumber("Drive/RR_omega_rps", rr.omegaRadiansPerSecond);
  }

  private void publishShooter() {
    SmartDashboard.putBoolean("Shooter/Ready", shooter.isAtSpeed());
    SmartDashboard.putNumber("Shooter/TopRPM", shooter.getTopRpm());
    SmartDashboard.putNumber("Shooter/MidRPM", shooter.getMidRpm());
    SmartDashboard.putNumber("Shooter/BottomRPM", shooter.getBottomRpm());

    double distM = FieldTargetUtil.distanceToHubMeters(swerve.getPose());
    SmartDashboard.putNumber("Shooter/HubDistanceM",  distM);
    SmartDashboard.putNumber("Shooter/HubDistanceIn", distM / 0.0254);
  }

  private void publishHood() {
    SmartDashboard.putNumber("Hood/PosRot", hood.getHoodRot());
    SmartDashboard.putNumber("Hood/TargetRot", hood.getTargetHoodRot());
    SmartDashboard.putBoolean("Hood/AtTarget", hood.atTarget());
  }

  private void publishIntake() {
    SmartDashboard.putBoolean("Intake/PivotHoldEnabled", intake.isPivotHoldEnabled());
    SmartDashboard.putBoolean("Intake/Open", intake.isOpen());

    SmartDashboard.putNumber("Intake/ThroughBoreContRot", intake.getThroughBoreContinuousRot());
    SmartDashboard.putNumber("Intake/PivotRot", intake.getPivotRot());
    SmartDashboard.putNumber("Intake/PivotTargetRot", intake.getPivotTargetRot());
    SmartDashboard.putNumber("Intake/PivotErrRot", intake.getPivotErrorRot());

    SmartDashboard.putNumber("Intake/PivotCmdPct", intake.getLastPivotPercent());
    SmartDashboard.putNumber("Intake/RollerCmdPct", intake.getLastRollerPercent());

    SmartDashboard.putNumber("Intake/PivotMotorRotorPosRot", intake.getPivotMotorRotorPosRot());
    SmartDashboard.putNumber("Intake/PivotMotorRotorVelRps", intake.getPivotMotorRotorVelRps());
    SmartDashboard.putNumber("Intake/RollerMotorRotorVelRps", intake.getRollerMotorRotorVelRps());
  }

  private void publishTransport() {
    SmartDashboard.putNumber("Transport/MainCmdPct", transport.getLastMainPercent());
    SmartDashboard.putNumber("Transport/FeederCmdPct", transport.getLastFeederPercent());

    SmartDashboard.putNumber("Transport/MainRotorPosRot", transport.getMainRotorPosRot());
    SmartDashboard.putNumber("Transport/MainRotorVelRps", transport.getMainRotorVelRps());
    SmartDashboard.putNumber("Transport/FeederRotorPosRot", transport.getFeederRotorPosRot());
    SmartDashboard.putNumber("Transport/FeederRotorVelRps", transport.getFeederRotorVelRps());
  }

  private void publishClimber() {
    SmartDashboard.putNumber("Climber/CmdPct", climber.getLastPercent());
    SmartDashboard.putNumber("Climber/RotorPosRot", climber.getRotorPosRot());
    SmartDashboard.putNumber("Climber/RotorVelRps", climber.getRotorVelRps());
  }

  private void publishVisionBasics() {
    SmartDashboard.putBoolean("Vision/HasTarget", vision.hasTarget());
    SmartDashboard.putBoolean("Vision/HasBotPoseBlue", vision.getBotPoseWpiBlue().isPresent());
    SmartDashboard.putNumber("Vision/LatencySec",
        vision.getBotPoseWpiBlue().map(m -> m.latencySeconds).orElse(0.0));
    SmartDashboard.putNumber("Vision/TagCount",
        vision.getBotPoseWpiBlue().map(m -> (double) m.tagCount).orElse(0.0));
  }

  private void publishSwerveModulesBestEffort() {
    SwerveDrive sd = swerve.getSwerveDrive();
    Object modules = tryInvoke(sd, "getModules");
    if (modules == null) modules = tryInvoke(sd, "getSwerveModules");
    if (modules == null) return;

    int count = getLength(modules);
    if (count <= 0) return;

    SmartDashboard.putNumber("Drive/ModuleCount", count);

    for (int i = 0; i < count; i++) {
      Object m = getIndex(modules, i);
      if (m == null) continue;

      Double driveVel = tryInvokeDouble(m, "getDriveVelocityMetersPerSecond");
      if (driveVel == null) driveVel = tryInvokeDouble(m, "getDriveVelocity");
      if (driveVel == null) driveVel = tryNestedStateSpeed(m);

      Double angleDeg = tryInvokeDouble(m, "getSteerAngleDegrees");
      if (angleDeg == null) angleDeg = tryInvokeDouble(m, "getAngleDegrees");
      if (angleDeg == null) angleDeg = tryNestedStateAngleDeg(m);

      Double absDeg = tryInvokeDouble(m, "getAbsoluteAngleDegrees");
      if (absDeg == null) absDeg = tryInvokeDouble(m, "getCanCoderDegrees");
      if (absDeg == null) absDeg = tryInvokeDouble(m, "getEncoderAbsolutePositionDegrees");

      String base = "Drive/Mod" + i + "/";

      if (driveVel != null) SmartDashboard.putNumber(base + "DriveVelMps", driveVel);
      if (angleDeg != null) SmartDashboard.putNumber(base + "AngleDeg", angleDeg);
      if (absDeg != null) SmartDashboard.putNumber(base + "AbsDeg", absDeg);
    }
  }

  private static Object tryInvoke(Object obj, String method) {
    try {
      Method m = obj.getClass().getMethod(method);
      return m.invoke(obj);
    } catch (Exception ignored) {
      return null;
    }
  }

  private static Double tryInvokeDouble(Object obj, String method) {
    Object o = tryInvoke(obj, method);
    if (o instanceof Number n) return n.doubleValue();
    return null;
  }

  private static int getLength(Object arrayOrList) {
    if (arrayOrList == null) return -1;
    if (arrayOrList.getClass().isArray()) return Array.getLength(arrayOrList);
    if (arrayOrList instanceof java.util.List<?> list) return list.size();
    return -1;
  }

  private static Object getIndex(Object arrayOrList, int i) {
    if (arrayOrList == null) return null;
    if (arrayOrList.getClass().isArray()) return Array.get(arrayOrList, i);
    if (arrayOrList instanceof java.util.List<?> list) return list.get(i);
    return null;
  }

  private static Double tryNestedStateSpeed(Object module) {
    try {
      Object state = tryInvoke(module, "getState");
      if (state == null) return null;
      Double speed = tryInvokeDouble(state, "getSpeedMetersPerSecond");
      if (speed != null) return speed;
    } catch (Exception ignored) {}
    return null;
  }

  private static Double tryNestedStateAngleDeg(Object module) {
    try {
      Object state = tryInvoke(module, "getState");
      if (state == null) return null;

      Object angle = tryInvoke(state, "angle");
      if (angle != null) {
        Double deg = tryInvokeDouble(angle, "getDegrees");
        if (deg != null) return deg;
      }

      Object rot = tryInvoke(state, "rotation");
      if (rot != null) {
        Double deg = tryInvokeDouble(rot, "getDegrees");
        if (deg != null) return deg;
      }
    } catch (Exception ignored) {}
    return null;
  }
}