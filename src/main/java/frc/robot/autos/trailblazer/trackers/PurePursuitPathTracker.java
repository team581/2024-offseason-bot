package frc.robot.autos.trailblazer.trackers;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autos.trailblazer.AutoPoint;
import java.util.List;

// TODO: Implement https://github.com/team581/2024-offseason-bot/issues/95
public class PurePursuitPathTracker implements PathTracker {
  private static final double LOOKAHEAD_DISTANCE = 1.0;
  private List<AutoPoint> points = List.of();
  private Pose2d currentRobotPose = new Pose2d();
  private double proximityRadius = 0.5;
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
    AutoPoint currentPoint = points.get(getCurrentPointIndex());
    currentTargetWaypoint = currentPoint.poseSupplier.get();
    double distanceToTarget =
        currentPose.getTranslation().getDistance(currentTargetWaypoint.getTranslation());
    if (currentPointIndex < points.size() - 1) {
      nextTargetWaypoint = points.get(currentPointIndex + 1).poseSupplier.get();
      if (distanceToTarget < proximityRadius) {
        currentPointIndex++;
      }
    }
    DogLog.log("Autos/Trailblazer/CurrentTargetPose", currentTargetWaypoint);
    DogLog.log("Autos/Trailblazer/DistanceToTarget", distanceToTarget);
    DogLog.log("Autos/Trailblazer/NextPointIndex", currentPointIndex);
  }

  @Override
  public Pose2d getTargetPose() {
    // TODO: Use for figuring out if a point is between the two waypoints for things like if the
    // robot is behind or in front of path or if lookahead too far
    // if (!((givenposex - startx) * (givenposex - endx) <= 0 && (givenposex - starty) * (givenposex
    // - endy) <= 0)) {}

    var x1 = currentTargetWaypoint.getX();
    var y1 = currentTargetWaypoint.getY();
    var x2 = nextTargetWaypoint.getX();
    var y2 = nextTargetWaypoint.getY();
    double pathSlope = (y2 - y1) / (x2 - x1);

    var perpendicularPoint =
        getPerpendicularPoint(currentTargetWaypoint, nextTargetWaypoint, currentRobotPose);
    var lookaheadPoint = getLookaheadPoint(perpendicularPoint, pathSlope, LOOKAHEAD_DISTANCE);
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

  private Pose2d getLookaheadPoint(Pose2d pointOnPath, double pathSlope, double lookaheadDistance) {
    double lookaheadPointX =
        pointOnPath.getX() + lookaheadDistance / Math.sqrt(1 + pathSlope * pathSlope);
    double lookaheadPointY =
        pointOnPath.getY() + pathSlope * (lookaheadDistance / Math.sqrt(1 + pathSlope * pathSlope));
    return new Pose2d(lookaheadPointX, lookaheadPointY, new Rotation2d());
  }
  ;
}
