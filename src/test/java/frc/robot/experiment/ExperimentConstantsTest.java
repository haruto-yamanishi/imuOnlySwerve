package frc.robot.experiment;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.experiment.ExperimentConstants.MotionSegment;
import java.util.List;
import org.junit.jupiter.api.Test;

class ExperimentConstantsTest {
  private static final double EPSILON = 1.0e-12;
  private static final int[][] INVERSE_PAIRS = {{1, 2}, {3, 4}, {5, 6}, {7, 8}};

  @Test
  void inverseMotionsHaveEqualDurationsAndOppositeCommands() {
    List<MotionSegment> segments = ExperimentConstants.MOTION_SEGMENTS;
    for (int[] pair : INVERSE_PAIRS) {
      MotionSegment forward = segments.get(pair[0]);
      MotionSegment reverse = segments.get(pair[1]);

      assertEquals(forward.durationSeconds(), reverse.durationSeconds(), EPSILON);
      assertEquals(forward.xSpeed(), -reverse.xSpeed(), EPSILON);
      assertEquals(forward.ySpeed(), -reverse.ySpeed(), EPSILON);
      assertEquals(forward.rotationSpeed(), -reverse.rotationSpeed(), EPSILON);
    }
  }

  @Test
  void idealTimeIntegralIsZeroForEveryAxis() {
    double integratedX = 0.0;
    double integratedY = 0.0;
    double integratedRotation = 0.0;

    for (MotionSegment segment : ExperimentConstants.MOTION_SEGMENTS) {
      integratedX += segment.xSpeed() * segment.durationSeconds();
      integratedY += segment.ySpeed() * segment.durationSeconds();
      integratedRotation += segment.rotationSpeed() * segment.durationSeconds();
    }

    assertEquals(0.0, integratedX, EPSILON);
    assertEquals(0.0, integratedY, EPSILON);
    assertEquals(0.0, integratedRotation, EPSILON);
  }

  @Test
  void sequenceIsContiguousAndStopsAtOrAfterEighteenSeconds() {
    List<MotionSegment> segments = ExperimentConstants.MOTION_SEGMENTS;
    assertEquals(0.0, segments.get(0).startTimeSeconds(), EPSILON);
    for (int index = 1; index < segments.size(); index++) {
      assertEquals(
          segments.get(index - 1).endTimeSeconds(),
          segments.get(index).startTimeSeconds(),
          EPSILON);
    }
    assertEquals(
        ExperimentConstants.EXPERIMENT_DURATION_SECONDS,
        segments.get(segments.size() - 1).endTimeSeconds(),
        EPSILON);

    assertStopped(ExperimentConstants.EXPERIMENT_DURATION_SECONDS);
    assertStopped(ExperimentConstants.EXPERIMENT_DURATION_SECONDS + 0.001);
    assertStopped(60.0);
  }

  private static void assertStopped(double timeSeconds) {
    MotionSegment segment = ExperimentConstants.motionSegmentAt(timeSeconds);
    assertEquals(0.0, segment.xSpeed(), EPSILON);
    assertEquals(0.0, segment.ySpeed(), EPSILON);
    assertEquals(0.0, segment.rotationSpeed(), EPSILON);
  }
}
