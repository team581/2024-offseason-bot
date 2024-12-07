package frc.robot.autos.trailblazer.trackers.pure_pursuit;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PurePursuitUtilsTest {
  @Test
  void getPerpindicularWaypoint() {
    var startPoint = new Pose2d(-4, 5, new Rotation2d());
    var endPoint = new Pose2d(1, 0, new Rotation2d());
    var robotPose = new Pose2d(0, 2, new Rotation2d());
    var result = PurePursuitUtils.getPerpendicularPoint(startPoint, endPoint, robotPose);

    var expected = new Pose2d(-0.5, 1.5, new Rotation2d());
    Assertions.assertEquals(expected, result);
  }

  @Test
  void getLookaheadWaypoint() {
    var startPoint = new Pose2d(4, 0, new Rotation2d());
    var endPoint = new Pose2d(4, 5, new Rotation2d());
    var pointOnPath = new Pose2d(4, 2, new Rotation2d());
    var result = PurePursuitUtils.getLookaheadPoint(startPoint, endPoint, pointOnPath, 1.0);

    var expected = new Pose2d(4, 3, new Rotation2d());
    Assertions.assertEquals(expected, result);
  }
}
