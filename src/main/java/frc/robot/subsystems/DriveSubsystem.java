package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import java.io.File;
import java.io.IOException;
import java.util.Optional;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;
  private final Field2d estimatedField = new Field2d();
  private SwerveModulePosition[] previousWheelPositions;
  private Pose2d estimatedPose = Pose2d.kZero;

  public DriveSubsystem() {
    swerveDrive = createSwerveDrive();
    previousWheelPositions = copyModulePositions(swerveDrive.getModulePositions());
    SmartDashboard.putData("IMU Only Pose", estimatedField);
  }

  @Override
  public void periodic() {
    estimatedPose = updateFromImuOnly();
    estimatedField.setRobotPose(estimatedPose);
    publishPose("ImuOnly", estimatedPose);
  }

  public void drive(double xSpeed, double ySpeed, double rotationSpeed) {
    swerveDrive.drive(
        new Translation2d(
            xSpeed * DriveConstants.kMaxSpeedMetersPerSecond,
            ySpeed * DriveConstants.kMaxSpeedMetersPerSecond),
        rotationSpeed * swerveDrive.getMaximumChassisAngularVelocity(),
        true,
        false);
  }

  public void stop() {
    drive(0.0, 0.0, 0.0);
  }

  public void resetPose(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
    estimatedPose = pose;
    previousWheelPositions = copyModulePositions(swerveDrive.getModulePositions());
  }

  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  public Optional<Pose2d> getSimulationGroundTruthPose() {
    return swerveDrive.getSimulationDriveTrainPose();
  }

  public void publishSimulatedCameraPose(Pose2d pose) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(VisionConstants.kCameraTable);
    table
        .getEntry(VisionConstants.kBotPoseEntry)
        .setDoubleArray(
            new double[] {
              pose.getX(), pose.getY(), 0.0, 0.0, 0.0, pose.getRotation().getDegrees()
            });
    table.getEntry("tl").setDouble(0.0);
    table.getEntry("cl").setDouble(0.0);
    table.getEntry("tv").setDouble(1.0);
  }

  private Pose2d updateFromWheelEncodersOnly() {
    SwerveModulePosition[] currentPositions = swerveDrive.getModulePositions();
    SwerveModulePosition[] deltas = new SwerveModulePosition[currentPositions.length];

    for (int i = 0; i < currentPositions.length; i++) {
      deltas[i] =
          new SwerveModulePosition(
              currentPositions[i].distanceMeters - previousWheelPositions[i].distanceMeters,
              currentPositions[i].angle);
    }

    previousWheelPositions = copyModulePositions(currentPositions);
    return estimatedPose.exp(swerveDrive.kinematics.toTwist2d(deltas));
  }

  private Pose2d updateFromImuOnly() {
    return new Pose2d(estimatedPose.getTranslation(), swerveDrive.getYaw());
  }

  @SuppressWarnings("unused")
  private Optional<VisionMeasurement> getLatestCameraPose() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(VisionConstants.kCameraTable);
    double[] botPose = table.getEntry(VisionConstants.kBotPoseEntry).getDoubleArray(new double[0]);
    if (botPose.length < 6) {
      return Optional.empty();
    }

    double latencySeconds =
        (table.getEntry("tl").getDouble(0.0) + table.getEntry("cl").getDouble(0.0)) / 1000.0;
    Pose2d pose =
        new Pose2d(
            botPose[0],
            botPose[1],
            Rotation2d.fromDegrees(botPose[5]));
    return Optional.of(new VisionMeasurement(pose, Timer.getFPGATimestamp() - latencySeconds));
  }

  private static SwerveDrive createSwerveDrive() {
    try {
      return new SwerveParser(new File(Filesystem.getDeployDirectory(), DriveConstants.kSwerveConfigDirectory))
          .createSwerveDrive(DriveConstants.kMaxSpeedMetersPerSecond, Pose2d.kZero);
    } catch (IOException exception) {
      DriverStation.reportError("Failed to create swerve drive: " + exception.getMessage(), exception.getStackTrace());
      throw new RuntimeException(exception);
    }
  }

  private static SwerveModulePosition[] copyModulePositions(SwerveModulePosition[] positions) {
    SwerveModulePosition[] copy = new SwerveModulePosition[positions.length];
    for (int i = 0; i < positions.length; i++) {
      copy[i] = new SwerveModulePosition(positions[i].distanceMeters, positions[i].angle);
    }
    return copy;
  }

  private static void publishPose(String prefix, Pose2d pose) {
    SmartDashboard.putNumber(prefix + "/X", pose.getX());
    SmartDashboard.putNumber(prefix + "/Y", pose.getY());
    SmartDashboard.putNumber(prefix + "/HeadingDeg", pose.getRotation().getDegrees());
  }

  private record VisionMeasurement(Pose2d pose, double timestampSeconds) {}
}
