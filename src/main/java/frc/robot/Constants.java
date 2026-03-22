package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.05;
    private OperatorConstants() {}
  }

  public static final class SwerveConstants {
    public static final double kTurnP = 5.0;
    public static final double kTurnI = 0.0;
    public static final double kTurnD = 0.2;

    public static final double kHoldHeadingP = 4.0;
    public static final double kHoldHeadingI = 0.0;
    public static final double kHoldHeadingD = 0.15;

    public static final double kTurnToleranceDeg = 2.0;
    public static final double kMaxAngularSpeedRadPerSec = Units.degreesToRadians(540.0);

    private SwerveConstants() {}
  }

  public static final class CanId {
    // Intake
    public static final int kIntakePivotKraken = 17;
    public static final int kIntakeRollerKraken = 19;

    // Transport (main conveyor inside robot, not shooter feeder)
    public static final int kMainTransportKraken = 16;
    public static final int kShooterFeederKraken = 20;

    // Shooter wheels
    public static final int kShooterTopKraken = 21;
    public static final int kShooterMidKraken = 22;
    public static final int kShooterBottomKraken = 23;

    // Climber
    public static final int kClimberKraken = 25;

    // Hood (NEO Vortex on Spark Flex)
    public static final int kHoodNeoVortex = 60;

    // Hood angle (NEO 1.1 on Spark Max)
    public static final int kHoodAngleNeo = 26;

    private CanId() {}
  }

  public static final class MotorInverts {
    public static final boolean kIntakePivotInverted = false;
    public static final boolean kIntakeRollerInverted = true;

    public static final boolean kMainTransportInverted = true;
    public static final boolean kShooterFeederInverted = false;

    public static final boolean kShooterTopInverted = false;
    public static final boolean kShooterMidInverted = true;
    public static final boolean kShooterBottomInverted = true;

    public static final boolean kClimberInverted = false;

    public static final boolean kHoodInverted = false;
    public static final boolean kHoodAngleInverted = false;

    private MotorInverts() {}
  }

  public static final class IntakeConstants {
    public static final int kThroughBoreDutyCycleDio = 2;

    // Pivot rotations per small sprocket rotation
    public static final double kSmallToPivotRatio = 1.0;

    /**
     * Absolute encoder zero offset in rotations [0,1).
     * Tune this once so the reported intake pivot position matches your real mechanism position
     * after reboot, instead of shifting every power cycle.
     */
    public static final double kPivotEncoderZeroOffsetRot = 0.0;

    /**
     * Intake target positions in encoder-rotation space after zero offset is applied.
     * These stay valid across reboots because they are referenced to kPivotEncoderZeroOffsetRot.
     */
    public static final double kClosedPivotRot = 0.54;
    public static final double kOpenPivotRot = 0.06;

    public static final double kPivotP = 1.2;
    public static final double kPivotI = 0.0;
    public static final double kPivotD = 0.03;

    public static final double kPivotMaxOut = 0.2;
    public static final double kPivotToleranceRot = 0.02;

    public static final double kRollerPercent = 0.80;

    private IntakeConstants() {}
  }

  public static final class TransportConstants {
    public static final double kTransportPercent = 0.70;
    public static final double kFeederOnlyPercent = 0.85;
    public static final double kConveyorOnlyPercent = 0.85;
    private TransportConstants() {}
  }

  public static final class ShooterConstants {
    public static final double kTopRpm = 4500.0;
    public static final double kMidRpm = 4500.0;
    public static final double kBottomRpm = 4500.0;

    public static final double kVelocityP = 1.0;
    public static final double kVelocityI = 2.0;
    public static final double kVelocityD = 0.0;
    public static final double kVelocityV = 12.0 / 100.0; // 12V at Kraken X60 free speed (100 RPS)
    public static final double kVelocityS = 0.0;
    // IZone: integrator only accumulates within ±500 RPM of target, preventing windup on startup/stall
    public static final double kVelocityIZone = 500.0 / 60.0; // 500 RPM in RPS (CTRE native units)

    public static final double kReadyRpmTolerance = 100.0;
    public static final double kReadyTimeSeconds = 0.20;

    public static final double kToggleTestLowRpm = 2600.0;
    public static final double kToggleTestHighRpm = 3900.0;

    private ShooterConstants() {}
  }

  /**
   * Hood is now a motor-driven joint.
   * Sensor is REV Through Bore duty-cycle encoder on DIO.
   *
   * Position units used here: "hood rotations" in the encoder's 0..1 scale.
   * That means:
   * - One full rotation of the encoder shaft = 1.0
   * - Your hood will likely use only part of that range.
   */
  public static final class HoodConstants {
    // NEO 1.1 on Spark Max
    public static final int kThroughBoreDio = 3;

    /**
     * Encoder zero offset: subtract from the raw absolute encoder reading
     * so that 0.0 corresponds to the lowest (fully closed) position.
     * Tune this from Elastic at runtime.
     */
    public static final double kEncoderOffsetRot = 0.0;

    /** Encoder position (after offset) for fully closed (lowest) position. */
    public static final double kClosedPos = 0.0;

    /** Encoder position (after offset) for fully open position. */
    public static final double kOpenPos = 0.3;

    /** Alias kept for SetHoodPreset compatibility. */
    public static final double kMinHoodRot = kClosedPos;

    public static final double kP = 6.0;
    public static final double kI = 0.0;
    public static final double kD = 0.2;

    public static final double kMaxOut = 0.5;
    public static final double kToleranceHoodRot = 0.005;

    private HoodConstants() {}
  }

  public static final class FeedConstants {
    public static final double kTransportPercent = 0.85;

    public static final double kJerkForwardPercent = 0.80;
    public static final double kJerkReversePercent = -0.35;
    public static final double kJerkForwardSeconds = 0.25;
    public static final double kJerkReverseSeconds = 0.10;

    private FeedConstants() {}
  }

  public static final class ClimberConstants {
    public static final double kClimbUpPercent = 0.60;
    public static final double kClimbDownPercent = -0.60;
    /** Stator current limit (amps) to protect the mechanism from stalling damage. */
    public static final double kStatorCurrentLimit = 60;
    private ClimberConstants() {}
  }

  /**
   * Unified distance-to-shot table.
   * Distances are stored in meters.
   * Hood values are stored in hood rotations.
   * Shooter values are stored in wheel RPM.
   *
   * Reference calibration points adapted from the WCP 2026 Competitive Concept architecture:
   * 52.0 in, hood 0.56, 2800 RPM
   * 114.4 in, hood 0.42, 3275 RPM
   * 165.5 in, hood 0.37, 3650 RPM
   */
  public static final class ShotLookup {
    public static final double[] kDistanceM = {
        Units.inchesToMeters(52.0),
        Units.inchesToMeters(114.4),
        Units.inchesToMeters(165.5)
    };

    public static final double[] kHoodRot = {0.56, 0.42, 0.37};

    public static final double[] kTopRpm = {2800.0, 3275.0, 3650.0};
    public static final double[] kMidRpm = {2800.0, 3275.0, 3650.0};
    public static final double[] kBottomRpm = {2800.0, 3275.0, 3650.0};

    private ShotLookup() {}
  }

  public static final class VisionConstants {
    public static final double kMaxPoseJumpMeters = 1.25;
    public static final double kMaxLatencySeconds = 0.25;
    public static final int kMinTagCount = 1;
    private VisionConstants() {}
  }

  public static final class PathPlannerConstants {
    public static final double kDriveBaseRadiusMeters = 0.45;
    private PathPlannerConstants() {}
  }

  public static final class AutoAimConstants {
    public static final double kUnderHubHeadingDeg = 0.0;
    private AutoAimConstants() {}
  }

  public static final double maxSpeed = Units.feetToMeters(14.5);

  private Constants() {}
}
