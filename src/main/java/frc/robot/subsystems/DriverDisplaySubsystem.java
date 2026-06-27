package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.FieldTargetUtil;

/**
 * Publishes only the relevant driver/bringup telemetry. Each mechanism subsystem owns and
 * publishes its own detail; this class covers robot health, drive pose, vision basics, and the
 * combined shoot-ready indicator. Kept lean on purpose — heavy per-loop publishing and
 * reflection-based module dumps were a major loop-time / CAN load source.
 */
public class DriverDisplaySubsystem extends SubsystemBase {

  private final SwerveSubsystem swerve;
  private final ShooterSubsystem shooter;
  private final HoodSubsystem hood;
  private final VisionSubsystem vision;

  private boolean readyToShoot = false;

  public DriverDisplaySubsystem(
      SwerveSubsystem swerve,
      ShooterSubsystem shooter,
      HoodSubsystem hood,
      VisionSubsystem vision) {
    this.swerve = swerve;
    this.shooter = shooter;
    this.hood = hood;
    this.vision = vision;
  }

  /** True only when the shooter is stably at RPM AND the hood is at angle — drives rumble + green
   *  light. Computed once per loop in {@link #periodic()}. */
  public boolean isReadyToShoot() {
    return readyToShoot;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Robot/BatteryV", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean("Robot/BrownedOut", RobotController.isBrownedOut());
    SmartDashboard.putNumber("Robot/CANUtil", RobotController.getCANStatus().percentBusUtilization);

    SmartDashboard.putNumber("Drive/PoseX", swerve.getPose().getX());
    SmartDashboard.putNumber("Drive/PoseY", swerve.getPose().getY());
    SmartDashboard.putNumber("Drive/HeadingDeg", swerve.getHeading().getDegrees());
    SmartDashboard.putNumber("Drive/DistToHubM", FieldTargetUtil.distanceToHubMeters(swerve.getPose()));

    SmartDashboard.putBoolean("Vision/HasTarget", vision.hasTarget());
    SmartDashboard.putNumber("Vision/TagCount",
        vision.getBotPoseWpiBlue().map(m -> (double) m.tagCount).orElse(0.0));

    // Green light for the driver: both subsystems ready (shooter stable + hood on angle).
    readyToShoot =
        shooter.isAtSpeedForTime(Constants.ShooterConstants.kReadyTimeSeconds) && hood.isAtAngle();
    SmartDashboard.putBoolean("Robot/ReadyToShoot", readyToShoot);
  }
}
