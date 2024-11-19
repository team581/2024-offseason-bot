package frc.robot.autos.trailblazer.trackers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autos.trailblazer.AutoPoint;
import java.util.List;

/**
 * A path tracker takes a list of points making up a path, and decides which intermediary point
 * along the path to drive to next.
 */
public interface PathTracker {
  /**
   * Reset the state of the path tracker, as well as set the new list of points to consider in
   * {@link #getTargetPose(Pose2d)}.
   *
   * @param points The new list of points to store.
   */
  public void resetAndSetPoints(List<AutoPoint> points);

  /**
   * Calculate the pose the robot should drive to next, based on its progress following the path.
   *
   * @param currentPose
   * @return
   */
  public Pose2d getTargetPose(Pose2d currentPose, ChassisSpeeds currentFieldRelativeRobotSpeeds);

  /**
   * Check whether the robot has finished following the given point.
   *
   * @param point The point to check.
   * @return Whether the robot has finished following the given point.
   */
  public boolean isFinished(AutoPoint point);
}
