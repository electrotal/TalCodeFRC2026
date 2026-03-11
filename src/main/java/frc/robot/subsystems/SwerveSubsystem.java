package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;

import java.io.File;
import java.nio.file.Files;
import java.util.HashSet;
import java.util.Set;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;
  private RobotConfig robotConfig = null;

  public SwerveSubsystem() {
    File deployDir = new File(Filesystem.getDeployDirectory(), "swerve");

    File configDir = deployDir;
    if (!configDir.exists() || !configDir.isDirectory()) {
      File fallback = new File("src/main/swerve");
      if (fallback.exists() && fallback.isDirectory()) {
        configDir = fallback;
        DriverStation.reportWarning(
            "YAGSL config not found in deploy/swerve. Using fallback src/main/swerve. Move it to src/main/deploy/swerve for the robot.",
            false);
      }
    }

    publishYagslModuleConfig(configDir);

    try {
      Pose2d initialPose = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0));
      swerveDrive = new SwerveParser(configDir).createSwerveDrive(Constants.maxSpeed, initialPose);
    } catch (Exception e) {
      throw new RuntimeException("Failed to create SwerveDrive from YAGSL config folder: " + configDir.getPath(), e);
    }

    configurePathPlanner();
  }

  private void configurePathPlanner() {
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      DriverStation.reportError("PathPlanner RobotConfig.fromGUISettings failed: " + e.getMessage(), e.getStackTrace());
      robotConfig = null;
      return;
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0)
        ),
        robotConfig,
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        },
        this
    );
  }

  // Publishes what IDs and offsets the code is using from the JSON files
  private void publishYagslModuleConfig(File configDir) {
    try {
      File modulesDir = new File(configDir, "modules");
      if (!modulesDir.exists() || !modulesDir.isDirectory()) {
        SmartDashboard.putString("Swerve/ConfigError", "No modules folder at: " + modulesDir.getPath());
        return;
      }

      ObjectMapper mapper = new ObjectMapper();
      String[] names = {"frontleft.json", "frontright.json", "backleft.json", "backright.json"};

      Set<Integer> driveIds = new HashSet<>();
      boolean dupDriveId = false;

      for (String n : names) {
        File f = new File(modulesDir, n);
        if (!f.exists()) {
          SmartDashboard.putString("Swerve/ConfigError", "Missing module file: " + f.getPath());
          continue;
        }

        String json = Files.readString(f.toPath());
        JsonNode root = mapper.readTree(json);

        int driveId = root.path("drive").path("id").asInt(-1);
        int angleId = root.path("angle").path("id").asInt(-1);
        int encId = root.path("encoder").path("id").asInt(-1);
        double offset = root.path("absoluteEncoderOffset").asDouble(0.0);

        SmartDashboard.putNumber("SwerveCfg/" + n + "/DriveId", driveId);
        SmartDashboard.putNumber("SwerveCfg/" + n + "/AngleId", angleId);
        SmartDashboard.putNumber("SwerveCfg/" + n + "/EncoderId", encId);
        SmartDashboard.putNumber("SwerveCfg/" + n + "/AbsOffsetDeg", offset);

        if (driveId != -1) {
          if (!driveIds.add(driveId)) dupDriveId = true;
        }
      }

      SmartDashboard.putBoolean("SwerveCfg/DuplicateDriveId", dupDriveId);

    } catch (Exception e) {
      SmartDashboard.putString("Swerve/ConfigError", "Config parse failed: " + e.getMessage());
    }
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void resetPose(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    swerveDrive.drive(speeds);
  }

  public Command driveFieldOriented(SwerveInputStream inputStream) {
    return run(() -> swerveDrive.driveFieldOriented(inputStream.get()));
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
    // Offset by 180° so the driver faces the robot's back during reset
    resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180.0)));
  }

  public boolean isVisionFusionSupported() {
    return true;
  }

  public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
    swerveDrive.addVisionMeasurement(pose, timestampSeconds);
  }
}