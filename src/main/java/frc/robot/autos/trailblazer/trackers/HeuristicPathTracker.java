package frc.robot.autos.trailblazer.trackers;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autos.trailblazer.AutoPoint;
import java.util.List;

// TODO: Implement https://github.com/team581/2024-offseason-bot/issues/94
public class HeuristicPathTracker implements PathTracker {
  private List<AutoPoint> points = List.of();
  private Pose2d currentPose = new Pose2d();
  private Pose2d currentTargetPose = new Pose2d();
  private double proximityRadius = 0.5;
  private double endPointProximityRadius = 0.1;
  private int currentPointIndex = 0;
  private double distanceToTarget;

  @Override
  public void resetAndSetPoints(List<AutoPoint> points) {
    this.points = points;
  }

  @Override
  public void updateRobotState(Pose2d currentPose, ChassisSpeeds currentFieldRelativeRobotSpeeds) {
    this.currentPose = currentPose;

    AutoPoint currentPoint = points.get(getCurrentPointIndex());
    Pose2d currentTargetPose = currentPoint.poseSupplier.get();

    double distanceToTarget =
        currentPose.getTranslation().getDistance(currentTargetPose.getTranslation());

    if (distanceToTarget < proximityRadius && currentPointIndex < points.size() - 1) {
      currentPointIndex++;
    }
    DogLog.log("Autos/Trailblazer/HeuristicPathTracker/CurrentTargetPose", currentTargetPose);
    DogLog.log("Autos/Trailblazer/HeuristicPathTracker/DistanceToTarget", distanceToTarget);
    DogLog.log("Autos/Trailblazer/HeuristicPathTracker/NextPointIndex", currentPointIndex);
  }

  @Override
  public Pose2d getTargetPose() {
    var targetPose = points.get(currentPointIndex).poseSupplier.get();

    DogLog.log("Autos/Trailblazer/HeuristicPathTracker/TargetPose", targetPose);

    return targetPose;
  }

  @Override
  public int getCurrentPointIndex() {
    return currentPointIndex;
  }

  @Override
  public boolean isFinished() {
    if (distanceToTarget <= endPointProximityRadius) {
      return true;
    }
    return false;
  }
}
