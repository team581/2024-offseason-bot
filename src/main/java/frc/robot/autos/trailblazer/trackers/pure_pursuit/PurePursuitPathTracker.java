package frc.robot.autos.trailblazer.trackers.pure_pursuit;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autos.trailblazer.AutoPoint;
import frc.robot.autos.trailblazer.trackers.PathTracker;
import java.util.List;

// TODO: Implement https://github.com/team581/2024-offseason-bot/issues/95
public class PurePursuitPathTracker implements PathTracker {
  private static final double LOOKAHEAD_DISTANCE = 1.0;
  private List<AutoPoint> points = List.of();
  private Pose2d currentRobotPose = new Pose2d();
  private Pose2d currentTargetWaypoint = new Pose2d();
  private Pose2d nextTargetWaypoint = new Pose2d();
  private int currentPointIndex = 0;

  @Override
  public void resetAndSetPoints(List<AutoPoint> points) {
    this.points = points;
  }

  @Override
  public void updateRobotState(Pose2d currentPose, ChassisSpeeds currentFieldRelativeRobotSpeeds) {
    this.currentRobotPose = currentPose;
    DogLog.log("Autos/Trailblazer/CurrentTargetPose", currentTargetWaypoint);
    DogLog.log("Autos/Trailblazer/NextPointIndex", currentPointIndex);
  }

  @Override
  public Pose2d getTargetPose() {
    if (points.size() < 2) {
      return points.get(0).poseSupplier.get();
    }

    currentTargetWaypoint = points.get(getCurrentPointIndex()).poseSupplier.get();
    if (getCurrentPointIndex() < points.size() - 1) {
      nextTargetWaypoint = points.get(getCurrentPointIndex() + 1).poseSupplier.get();
    } else {
      return points.get(getCurrentPointIndex()).poseSupplier.get();
    }
    var startX = currentTargetWaypoint.getX();
    var startY = currentTargetWaypoint.getY();
    var endX = nextTargetWaypoint.getX();
    var endY = nextTargetWaypoint.getY();
    var perpendicularPoint =
        getPerpendicularPoint(currentTargetWaypoint, nextTargetWaypoint, currentRobotPose);
    var lookaheadPoint =
        getLookaheadPoint(
            currentTargetWaypoint, nextTargetWaypoint, perpendicularPoint, LOOKAHEAD_DISTANCE);

    var lookaheadX = lookaheadPoint.getX();
    var lookaheadY = lookaheadPoint.getY();
    // Check if lookahead point is outside of the line segment
    if (!((lookaheadX - startX) * (lookaheadX - endX) <= 0
        && (lookaheadY - startY) * (lookaheadY - endY) <= 0)) {

      var distanceToStart =
          Math.sqrt(Math.pow(lookaheadX - startX, 2) + Math.pow(lookaheadY - startY, 2));
      var distanceToEnd =
          Math.sqrt(Math.pow(lookaheadX - endX, 2) + Math.pow(lookaheadY - endY, 2));
      // Check if lookaheadpoint is outside on the starting side of the line
      if (distanceToStart < distanceToEnd) {
        // Target pose just becomes start point + lookahead
        return getLookaheadPoint(
            currentTargetWaypoint, nextTargetWaypoint, currentTargetWaypoint, LOOKAHEAD_DISTANCE);
      }

      // Otherwise, the lookahead point is past the end point, which means we need to go around
      // corner.
      // TODO: Fix for case of perp point is also past end point
      var perpDistanceToEnd =
          Math.sqrt(
              Math.pow(perpendicularPoint.getX() - endX, 2)
                  + Math.pow(perpendicularPoint.getY() - endY, 2));
      // check if we're at corner
      if (getCurrentPointIndex() < points.size() - 2) {
        var futurePoint = points.get(getCurrentPointIndex() + 2).poseSupplier.get();
        currentPointIndex++;
        return getLookaheadPoint(
            nextTargetWaypoint,
            futurePoint,
            nextTargetWaypoint,
            LOOKAHEAD_DISTANCE - perpDistanceToEnd);
      }
      // otherwise just return the end point
      else {
        return new Pose2d(endX, endY, new Rotation2d());
      }
    }
    return lookaheadPoint;
  }

  @Override
  public int getCurrentPointIndex() {
    return currentPointIndex;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private Pose2d getPerpendicularPoint(Pose2d startPoint, Pose2d endPoint, Pose2d robotPose) {
    var x1 = startPoint.getX();
    var y1 = startPoint.getY();

    var x2 = endPoint.getX();
    var y2 = endPoint.getY();

    var x3 = robotPose.getX();
    var y3 = robotPose.getY();

    if (y1 == y2) {
      return new Pose2d(x3, y1, new Rotation2d());
    }
    if (x1 == x2) {
      return new Pose2d(x1, y3, new Rotation2d());
    }
    // Find the slope of the path and y-int
    double pathSlope = (y2 - y1) / (x2 - x1);
    double yInt = y1 - pathSlope * x1;

    // Find the slope and y-int of the perpendicular line
    double perpSlope = -1 / pathSlope;
    double perpYInt = y3 - perpSlope * x3;

    // Calculate the perpendicular intersection
    double perpX = (perpYInt - yInt) / (pathSlope - perpSlope);
    double perpY = pathSlope * perpX + yInt;

    return new Pose2d(perpX, perpY, new Rotation2d());
  }

  private Pose2d getLookaheadPoint(
      Pose2d startPoint, Pose2d endPoint, Pose2d pointOnPath, double lookaheadDistance) {
    var x1 = startPoint.getX();
    var y1 = startPoint.getY();

    var x2 = endPoint.getX();
    var y2 = endPoint.getY();

    var x = pointOnPath.getX();
    var y = pointOnPath.getY();

    var xLookahead =
        x
            + lookaheadDistance
                * ((x2 - x1) / (Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2))));
    var yLookahead =
        y
            + lookaheadDistance
                * ((y2 - y1) / (Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2))));
    return new Pose2d(xLookahead, yLookahead, new Rotation2d());
  }
}