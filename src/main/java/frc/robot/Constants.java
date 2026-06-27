package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.175;
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
    public static final int kClimberKraken = 26;

    // Hood (NEO 1.1 on Spark Max)
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
    public static final double kClosedPivotRot = 0.78;
    public static final double kOpenPivotRot = 0.36;
    public static final double kClopenPivotRot = 0.55;

    /** Intake jerk: the pivot oscillates between clopen and this "half-clopen" position (wheels
     *  stay off) to settle balls. Tune the position and dwell to taste. */
    public static final double kJerkHalfClopenRot = 0.45;
    public static final double kJerkDwellSeconds = 0.2;

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
    public static final double kVelocityI = 0.0;
    public static final double kVelocityD = 0.0;
    public static final double kVelocityV = 12.0 / 100.0;
    public static final double kVelocityS = 0.0;

    public static final double kReadyRpmTolerance = 120.0;
    public static final double kReadyTimeSeconds = 0.20;

    public static final double kToggleTestLowRpm = 2500.0;
    public static final double kToggleTestHighRpm = 5000.0;

    /** Point-blank hub preset RPM (POV-Right). Tune for your point-blank shot. */
    public static final double kHubPresetRpm = 2800.0;

    private ShooterConstants() {}
  }

  /**
   * Hood = NEO 1.1 on a Spark MAX (CAN {@link CanId#kHoodAngleNeo}). Position comes from a REV
   * Through-Bore absolute encoder on RoboRIO DIO {@link #kThroughBoreDio} (duty-cycle), read via
   * {@link frc.robot.util.MultiTurnAbsoluteEncoder} — the same proven path the intake uses.
   *
   * Units: "hood rotations". The encoder is geared faster than the hood
   * ({@link #kEncoderTurnsPerHoodTurn}), and full travel exceeds one encoder turn, so a multi-turn
   * accumulator gives a continuous reading. Closed = 0; the open limit is measured on the real
   * mechanism (see the distance-calibration / hood-zero procedure) and stays tunable.
   *
   * Mutable fields are live-tunable from Elastic — they are the calibration knobs.
   */
  public static final class HoodConstants {
    /** REV Through-Bore absolute encoder on a RoboRIO DIO channel (duty-cycle). */
    public static final int kThroughBoreDio = 6;

    /** Encoder shaft turns per one hood rotation (encoder geared ~2x the hood). */
    public static final double kEncoderTurnsPerHoodTurn = 2.0;

    /** Invert the absolute encoder if its reading counts down as the hood opens. */
    public static final boolean kAbsEncoderInverted = false;

    /** Software zero offset in ENCODER rotations. The Hood/ZeroNow button overwrites this at
     *  runtime by capturing the current reading at the closed stop (no hardware reset). */
    public static double kEncoderOffsetRot = 0.0;

    /** Hood fully closed (steepest). Hard min for clamping. */
    public static final double kClosedHoodRot = 0.00;
    public static final double kMinHoodRot = kClosedHoodRot;
    /** Hood fully open (flattest), in hood rotations from closed. Measured: ~0.07. Tune live. */
    public static double kOpenHoodRot = 0.07;

    /** Gentle position PID — live-tunable from Elastic. */
    public static double kP = 6.0;
    public static double kI = 0.0;
    public static double kD = 0.2;

    /** Gentleness guards (live-tunable): output magnitude cap and per-loop slew cap. */
    public static double kMaxOut = 0.30;
    public static double kSlewPerLoop = 0.04;

    /** Point-blank hub preset hood angle (POV-Right) — min/steep angle for a point-blank shot. */
    public static final double kHubPresetHoodRot = kClosedHoodRot;

    public static final double kToleranceHoodRot = 0.005;

    private HoodConstants() {}
  }

  public static final class FeedConstants {
    public static final double kTransportPercent = 0.85;

    public static final double kJerkForwardPercent = 0.80;
    public static final double kJerkReversePercent = -0.35;
    public static final double kJerkForwardSeconds = 0.25;
    public static final double kJerkReverseSeconds = 0.10;
    public static final double kHoodRangeMin = 0;
    public static final double kHoodRangeMax = 1.25;

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
    // Distances must be in ascending order for interpolation to work correctly
    public static final double[] kDistanceM = {
        Units.inchesToMeters(52.0),   // ~1.32 m
        Units.inchesToMeters(114.4),  // ~2.91 m
        3.80,                         //  3.80 m
        Units.inchesToMeters(165.5)   // ~4.20 m
    };

    public static final double[] kHoodRot = {0.56, 0.42, 0.39, 0.37};

    public static final double[] kTopRpm    = {2800.0, 3275.0, 3900.0, 3650.0};
    public static final double[] kMidRpm    = {2800.0, 3275.0, 3900.0, 3650.0};
    public static final double[] kBottomRpm = {2800.0, 3275.0, 3900.0, 3650.0};

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
