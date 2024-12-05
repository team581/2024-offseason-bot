package frc.robot.autos.trailblazer.trackers.pure_pursuit;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class PurePursuitUtilsTest {
  @Test
  void addTwoNumbers() {
    var a = 1.0;
    var b = 3.0;

    var result = PurePursuitUtils.add(a, b);
    var expected = 4.0;

    Assertions.assertEquals(expected, result);
  }

  @Test
  void getPerpindicularWaypoint() {
    var startPoint = new Pose2d(-4, 5, new Rotation2d());
    var endPoint = new Pose2d(1, 0, new Rotation2d());
    var robotPose = new Pose2d(0, 2, new Rotation2d());
    var result = PurePursuitUtils.getPerpendicularPoint(startPoint, endPoint, robotPose);

    var expectedX = -0.5;
    var expectedY = 1.5;
    var resultX = result.getX();
    var resultY = result.getY();
    Assertions.assertEquals(expectedX, resultX);
    Assertions.assertEquals(expectedY, resultY);
  }

  @Test
  void getLookaheadWaypoint() {
    var startPoint = new Pose2d(4, 0, new Rotation2d());
    var endPoint = new Pose2d(4, 5, new Rotation2d());
    var pointOnPath = new Pose2d(4, 2, new Rotation2d());
    var result = PurePursuitUtils.getLookaheadPoint(startPoint, endPoint, pointOnPath, 1.0);

    var expectedX = 4;
    var expectedY = 3;
    var resultX = result.getX();
    var resultY = result.getY();
    Assertions.assertEquals(expectedX, resultX);
    Assertions.assertEquals(expectedY, resultY);
  }
}
