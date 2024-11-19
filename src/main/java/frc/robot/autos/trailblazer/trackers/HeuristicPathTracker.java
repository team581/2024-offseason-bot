package frc.robot.autos.trailblazer.trackers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autos.trailblazer.AutoPoint;
import java.util.List;

// TODO: Implement https://github.com/team581/2024-offseason-bot/issues/94
public class HeuristicPathTracker implements PathTracker {
  private List<AutoPoint> points = List.of();

  @Override
  public void resetAndSetPoints(List<AutoPoint> points) {
    this.points = points;
  }

  @Override
  public Pose2d getTargetPose(Pose2d currentPose, ChassisSpeeds currentFieldRelativeRobotSpeeds) {
    return null;
  }

  @Override
  public AutoPoint getCurrentPoint() {
    return null;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
