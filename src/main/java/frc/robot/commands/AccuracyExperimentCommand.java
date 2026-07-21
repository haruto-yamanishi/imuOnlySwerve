package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.experiment.ExperimentConstants;
import frc.robot.experiment.ExperimentConstants.MotionSegment;
import frc.robot.subsystems.DriveSubsystem;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Optional;

public class AccuracyExperimentCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final Timer timer = new Timer();
  private PrintWriter csvWriter;
  private double lastSimulatedVisionTimeSeconds = -1.0;
  private double translationErrorSumSquares;
  private double headingErrorSumSquares;
  private double maxTranslationErrorMeters;
  private double maxHeadingErrorDegrees;
  private int samples;

  public AccuracyExperimentCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.resetPose(ExperimentConstants.START_POSE);
    timer.restart();
    lastSimulatedVisionTimeSeconds = -1.0;
    translationErrorSumSquares = 0.0;
    headingErrorSumSquares = 0.0;
    maxTranslationErrorMeters = 0.0;
    maxHeadingErrorDegrees = 0.0;
    samples = 0;
    openCsv();
  }

  @Override
  public void execute() {
    double timeSeconds = timer.get();
    MotionSegment segment = ExperimentConstants.motionSegmentAt(timeSeconds);
    driveSubsystem.driveRobotRelative(
        segment.xSpeed(), segment.ySpeed(), segment.rotationSpeed());

    if (RobotBase.isSimulation()) {
      publishSimulatedVisionIfReady(timeSeconds);
    }

    Pose2d estimatedPose = driveSubsystem.getEstimatedPose();
    Optional<Pose2d> groundTruthPose = driveSubsystem.getSimulationGroundTruthPose();
    groundTruthPose.ifPresent(truth -> recordSample(timeSeconds, estimatedPose, truth));

    SmartDashboard.putNumber(ExperimentConstants.PROJECT_NAME + "/ExperimentTimeSec", timeSeconds);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
    timer.stop();
    if (csvWriter != null) {
      csvWriter.flush();
      csvWriter.close();
      csvWriter = null;
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(ExperimentConstants.EXPERIMENT_DURATION_SECONDS);
  }

  private void publishSimulatedVisionIfReady(double timeSeconds) {
    if (timeSeconds - lastSimulatedVisionTimeSeconds
        < ExperimentConstants.SIMULATED_CAMERA_PERIOD_SECONDS) {
      return;
    }

    driveSubsystem
        .getSimulationGroundTruthPose()
        .ifPresent(
            truth -> {
              driveSubsystem.publishSimulatedCameraPose(truth);
              lastSimulatedVisionTimeSeconds = timeSeconds;
            });
  }

  private void recordSample(double timeSeconds, Pose2d estimatedPose, Pose2d groundTruthPose) {
    double translationErrorMeters =
        estimatedPose.getTranslation().getDistance(groundTruthPose.getTranslation());
    double headingErrorRadians =
        MathUtil.angleModulus(
            estimatedPose.getRotation().minus(groundTruthPose.getRotation()).getRadians());
    double headingErrorDegrees = Math.toDegrees(Math.abs(headingErrorRadians));

    translationErrorSumSquares += translationErrorMeters * translationErrorMeters;
    headingErrorSumSquares += headingErrorRadians * headingErrorRadians;
    maxTranslationErrorMeters = Math.max(maxTranslationErrorMeters, translationErrorMeters);
    maxHeadingErrorDegrees = Math.max(maxHeadingErrorDegrees, headingErrorDegrees);
    samples++;

    double rmsTranslationErrorMeters = Math.sqrt(translationErrorSumSquares / samples);
    double rmsHeadingErrorDegrees = Math.toDegrees(Math.sqrt(headingErrorSumSquares / samples));

    String prefix = ExperimentConstants.PROJECT_NAME + "/Accuracy";
    SmartDashboard.putNumber(prefix + "/TranslationErrorMeters", translationErrorMeters);
    SmartDashboard.putNumber(prefix + "/HeadingErrorDegrees", headingErrorDegrees);
    SmartDashboard.putNumber(prefix + "/RmsTranslationErrorMeters", rmsTranslationErrorMeters);
    SmartDashboard.putNumber(prefix + "/RmsHeadingErrorDegrees", rmsHeadingErrorDegrees);
    SmartDashboard.putNumber(prefix + "/MaxTranslationErrorMeters", maxTranslationErrorMeters);
    SmartDashboard.putNumber(prefix + "/MaxHeadingErrorDegrees", maxHeadingErrorDegrees);

    if (csvWriter != null) {
      csvWriter.printf(
          "%.3f,%.5f,%.5f,%.3f,%.5f,%.5f,%.3f,%.5f,%.3f%n",
          timeSeconds,
          estimatedPose.getX(),
          estimatedPose.getY(),
          estimatedPose.getRotation().getDegrees(),
          groundTruthPose.getX(),
          groundTruthPose.getY(),
          groundTruthPose.getRotation().getDegrees(),
          translationErrorMeters,
          headingErrorDegrees);
    }
  }

  private void openCsv() {
    try {
      File csvFile =
          new File(
              Filesystem.getOperatingDirectory(),
              ExperimentConstants.PROJECT_NAME + "-pose-accuracy.csv");
      csvWriter = new PrintWriter(new FileWriter(csvFile));
      csvWriter.println(
          "timeSec,estimatedX,estimatedY,estimatedHeadingDeg,truthX,truthY,truthHeadingDeg,translationErrorMeters,headingErrorDegrees");
      SmartDashboard.putString(
          ExperimentConstants.PROJECT_NAME + "/Accuracy/CsvPath", csvFile.getAbsolutePath());
    } catch (IOException exception) {
      DriverStation.reportWarning("Could not open pose accuracy CSV: " + exception.getMessage(), false);
      csvWriter = null;
    }
  }
}
