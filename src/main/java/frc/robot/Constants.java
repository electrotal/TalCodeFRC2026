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
    public static final int kIntakePivotKraken = 17; // you provided
    public static final int kIntakeRollerKraken = 19;

    // Transport (main conveyor inside robot, not shooter feeder)
    public static final int kMainTransportKraken = 16; // you provided
    public static final int kShooterFeederKraken = 20;

    // Shooter wheels
    public static final int kShooterTopKraken = 40;    // set real value
    public static final int kShooterMidKraken = 41;    // set real value
    public static final int kShooterBottomKraken = 42; // set real value

    // Climber
    public static final int kClimberKraken = 50; // set real value

    // Hood (NEO Vortex on Spark Flex)
    public static final int kHoodNeoVortex = 60; // set real value

    private CanId() {}
  }

  public static final class MotorInverts {
    public static final boolean kIntakePivotInverted = false;
    public static final boolean kIntakeRollerInverted = true;

    public static final boolean kMainTransportInverted = true;
    public static final boolean kShooterFeederInverted = false;

    public static final boolean kShooterTopInverted = false;
    public static final boolean kShooterMidInverted = false;
    public static final boolean kShooterBottomInverted = false;

    public static final boolean kClimberInverted = false;

    public static final boolean kHoodInverted = false;

    private MotorInverts() {}
  }


  public static final class IntakeConstants {
    public static final int kThroughBoreDutyCycleDio = 0; // set real DIO

    // Pivot rotations per small sprocket rotation
    public static final double kSmallToPivotRatio = 1.0; // set from sprockets

    public static final double kClosedPivotRot = 0.54; // measure
    public static final double kOpenPivotRot = 0.06;  // measure

    public static final double kPivotP = 1.0; // tune
    public static final double kPivotI = 0.0;
    public static final double kPivotD = 0.02;
    public static final double kPivotMaxOut = 0.15;

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

    public static final double kVelocityP = 0.12;
    public static final double kVelocityI = 0.0;
    public static final double kVelocityD = 0.0;

    public static final double kReadyRpmTolerance = 100.0;
    public static final double kReadyTimeSeconds = 0.20;

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
    // ⚙️ Spark Flex CAN ID for the hood motor
    public static final int kHoodMotorCanId = CanId.kHoodNeoVortex;

    // ⚙️ Through Bore duty-cycle encoder DIO channel
    public static final int kThroughBoreDio = 1;

    /**
     * ⚙️ If the encoder is geared, set how many HOOD rotations happen per 1 encoder rotation.
     * - Direct on hood shaft: 1.0
     * - Example: encoder turns 3 times while hood turns 1 time => hood/encoder = 1/3 => 0.3333
     */
    public static final double kEncoderToHoodRatio = 1.0;

    /**
     * ⚙️ Encoder zero offset in ENCODER rotations (not hood rotations).
     * Measure encoder reading at your desired hood-zero, set this so hood position becomes ~0 at that point.
     */
    public static final double kEncoderOffsetRot = 0.0;

    // ⚙️ Hood soft limits in HOOD rotations (after ratio+offset)
    public static final double kMinHoodRot = 0.00;
    public static final double kMaxHoodRot = 0.35;

    // 🛞 Hood PID (software PID driving motor percent)
    public static final double kP = 6.0;
    public static final double kI = 0.0;
    public static final double kD = 0.2;

    // ⚙️ Clamp motor output for safety
    public static final double kMaxOut = 0.5;

    // ⚙️ Consider hood at target within this tolerance (hood rotations)
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
    public static final double kClimbUpPercent = 0.70;
    public static final double kClimbDownPercent = -0.50;
    private ClimberConstants() {}
  }

  /**
   * Shot lookup tables:
   * distance (meters) -> hood position (encoder rotations 0..1) + shooter RPMs.
   * You will fill these after collecting data.
   */
  public static final class ShotLookup {
    public static final double[] kDistanceM = {1.5, 3.0, 4.5, 6.0};

    // ⚙️ Hood targets in HOOD rotations
    public static final double[] kHoodRot = {0.10, 0.16, 0.22, 0.28};

    public static final double[] kTopRpm = {4200.0, 4500.0, 4800.0, 5200.0};
    public static final double[] kMidRpm = {4200.0, 4500.0, 4800.0, 5200.0};
    public static final double[] kBottomRpm = {4200.0, 4500.0, 4800.0, 5200.0};

  private ShotLookup() {}
}

  public static final class VisionConstants {
    public static final double kMaxPoseJumpMeters = 1.25;
    public static final double kMaxLatencySeconds = 0.25;
    public static final int kMinTagCount = 2;
    private VisionConstants() {}
  }

  public static final class PathPlannerConstants {
    public static final double kDriveBaseRadiusMeters = 0.45; // set from CAD
    private PathPlannerConstants() {}
  }

  public static final class AutoAimConstants {
    public static final double kUnderHubHeadingDeg = 0.0; // measure
    private AutoAimConstants() {}
  }

  public static final double maxSpeed = Units.feetToMeters(14.5);

  private Constants() {}
}