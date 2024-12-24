package frc.robot.autos.trailblazer.trackers.pure_pursuit;

import java.util.List;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autos.trailblazer.AutoPoint;
import frc.robot.autos.trailblazer.trackers.PathTracker;

// TODO: Implement https://github.com/team581/2024-offseason-bot/issues/95
public class PurePursuitPathTracker implements PathTracker {
  private static final double LOOKAHEAD_DISTANCE = 1.0;
  private static final double FINISHED_THRESHOLD = 0.05;
  private List<AutoPoint> points = List.of();
  private Pose2d currentRobotPose = new Pose2d();
  private Pose2d startingRobotPose = new Pose2d();
  private boolean startingRobotPoseUpdated = false;
  private Pose2d currentTargetWaypoint = new Pose2d();
  private Pose2d nextTargetWaypoint = new Pose2d();
  private int currentPointIndex = 0;

  @Override
  public void resetAndSetPoints(List<AutoPoint> points) {
    startingRobotPose = new Pose2d();
    startingRobotPoseUpdated = false;
    currentPointIndex = 0;
    this.points = points;
  }

  @Override
  public void updateRobotState(Pose2d currentPose, ChassisSpeeds currentFieldRelativeRobotSpeeds) {
    this.currentRobotPose = currentPose;
    DogLog.log(
        "Autos/Trailblazer/PurePursuitPathTracker/CurrentPointIndex", getCurrentPointIndex());
    DogLog.log(
        "Autos/Trailblazer/PurePursuitPathTracker/StartingRobotPose/Point", startingRobotPose);
    DogLog.log(
        "Autos/Trailblazer/PurePursuitPathTracker/StartingRobotPose/Updated",
        startingRobotPoseUpdated);

    if (!startingRobotPoseUpdated) {
      startingRobotPose = currentPose;
      startingRobotPoseUpdated = true;
    }
  }

  @Override
  public Pose2d getTargetPose() {
    DogLog.log("Autos/Trailblazer/PurePursuitPathTracker/Size", points.size());

    if (points.isEmpty()) {
      return new Pose2d();
    }

    if (points.size() == 1) {
      currentTargetWaypoint = startingRobotPose;
    } else {
      currentTargetWaypoint = points.get(getCurrentPointIndex()).poseSupplier.get();
    }

    if (getCurrentPointIndex() < points.size() - 1) {
      nextTargetWaypoint = points.get(getCurrentPointIndex() + 1).poseSupplier.get();
    } else {
      nextTargetWaypoint = points.get(getCurrentPointIndex()).poseSupplier.get();
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
      // Check if lookaheadpoint is outside on the ending side of the line
      if (distanceToStart > distanceToEnd) {
        var perpDistanceToEnd =
            Math.sqrt(
                Math.pow(perpendicularPoint.getX() - endX, 2)
                    + Math.pow(perpendicularPoint.getY() - endY, 2));
        // check if we're at corner
        if (getCurrentPointIndex() < points.size() - 2) {
          var futurePoint = points.get(getCurrentPointIndex() + 2).poseSupplier.get();
          currentPointIndex++;
          var targetPose =
              getLookaheadPoint(
                  nextTargetWaypoint,
                  futurePoint,
                  nextTargetWaypoint,
                  LOOKAHEAD_DISTANCE - perpDistanceToEnd);
          DogLog.log("Autos/Trailblazer/PurePursuitPathTracker/TargetPose", targetPose);
          return targetPose;

        }
        // otherwise just return the end point
        else {
          var targetPose = new Pose2d(endX, endY, getPointToPointInterpolatedRotation(currentTargetWaypoint, nextTargetWaypoint,  currentRobotPose));
          DogLog.log("Autos/Trailblazer/PurePursuitPathTracker/TargetPose", targetPose);
          return targetPose;
        }
      } else {
        var targetPose =
            getLookaheadPoint(
                startingRobotPose,
                currentTargetWaypoint,
                getPerpendicularPoint(startingRobotPose, currentTargetWaypoint, currentRobotPose),
                LOOKAHEAD_DISTANCE);
        DogLog.log("Autos/Trailblazer/PurePursuitPathTracker/TargetPose", targetPose);
        return targetPose;
      }
    }
    var targetPose = lookaheadPoint;
    DogLog.log("Autos/Trailblazer/PurePursuitPathTracker/TargetPose", targetPose);
    return targetPose;
  }

  @Override
  public int getCurrentPointIndex() {
    return currentPointIndex;
  }

  @Override
  public boolean isFinished() {
    if (points.isEmpty()) {
      return true;
    }
    if (currentRobotPose
            .getTranslation()
            .getDistance(points.get(points.size() - 1).poseSupplier.get().getTranslation())
        < FINISHED_THRESHOLD) {
      return true;
    }
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
    return new Pose2d(xLookahead, yLookahead, getPointToPointInterpolatedRotation(startPoint,  endPoint,  pointOnPath));
  }


  private Rotation2d getPointToPointInterpolatedRotation(Pose2d startPoint, Pose2d endPoint, Pose2d pointOnPath) {
    //TODO: do unit tests for interpolated rotation
  var totalDistance = startPoint.getTranslation().getDistance(endPoint.getTranslation());
  var pointToStart = pointOnPath.getTranslation().getDistance(startPoint.getTranslation());
  var pointToEnd = pointOnPath.getTranslation().getDistance(endPoint.getTranslation());

  if (!((pointOnPath.getX() - startPoint.getX()) * (pointOnPath.getX() - endPoint.getX()) <= 0
      && (pointOnPath.getY() - startPoint.getY()) * (pointOnPath.getY() - endPoint.getY()) <= 0)) {
    if (pointToEnd > pointToStart) {
      return startPoint.getRotation();
    } else {
      return endPoint.getRotation();
    }
  }
  var progressPercent = Math.abs((pointToStart / totalDistance));
  if (progressPercent > 0.8) {
    progressPercent = 1.0;
  }

  var interpolatedRotation = startPoint.getRotation().interpolate(endPoint.getRotation(), progressPercent);

  return interpolatedRotation;

   }
}
