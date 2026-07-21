package frc.robot.experiment;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;

/** Constants shared by every phase of the pose accuracy experiment. */
public final class ExperimentConstants {
  public static final String PROJECT_NAME = "imuOnlySwerve";
  public static final double SIMULATED_CAMERA_PERIOD_SECONDS = 0.10;

  public static final Pose2d START_POSE = Pose2d.kZero;
  public static final double EXPERIMENT_DURATION_SECONDS = 18.0;

  /** Robot-relative normalized commands. Each time interval is [start, end). */
  public static final List<MotionSegment> MOTION_SEGMENTS =
      List.of(
          new MotionSegment(0.0, 1.0, 0.0, 0.0, 0.0),
          new MotionSegment(1.0, 3.0, 0.20, 0.0, 0.0),
          new MotionSegment(3.0, 5.0, -0.20, 0.0, 0.0),
          new MotionSegment(5.0, 7.0, 0.0, 0.18, 0.0),
          new MotionSegment(7.0, 9.0, 0.0, -0.18, 0.0),
          new MotionSegment(9.0, 11.0, 0.0, 0.0, 0.18),
          new MotionSegment(11.0, 13.0, 0.0, 0.0, -0.18),
          new MotionSegment(13.0, 15.0, 0.16, 0.12, 0.0),
          new MotionSegment(15.0, 17.0, -0.16, -0.12, 0.0),
          new MotionSegment(17.0, EXPERIMENT_DURATION_SECONDS, 0.0, 0.0, 0.0));

  private static final MotionSegment STOPPED_AFTER_EXPERIMENT =
      new MotionSegment(
          EXPERIMENT_DURATION_SECONDS,
          Double.POSITIVE_INFINITY,
          0.0,
          0.0,
          0.0);

  public static MotionSegment motionSegmentAt(double timeSeconds) {
    for (MotionSegment segment : MOTION_SEGMENTS) {
      if (segment.contains(timeSeconds)) {
        return segment;
      }
    }
    return STOPPED_AFTER_EXPERIMENT;
  }

  public record MotionSegment(
      double startTimeSeconds,
      double endTimeSeconds,
      double xSpeed,
      double ySpeed,
      double rotationSpeed) {
    public double durationSeconds() {
      return endTimeSeconds - startTimeSeconds;
    }

    private boolean contains(double timeSeconds) {
      return timeSeconds >= startTimeSeconds && timeSeconds < endTimeSeconds;
    }
  }

  private ExperimentConstants() {}
}
