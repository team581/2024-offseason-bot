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
   * Get the input auto point that is currently most relevant to whatever the tracker is doing. Used
   * for triggering side effects.
   *
   * @return The point being tracked.
   */
  public AutoPoint getCurrentPoint();

  /**
   * Check whether the robot has finished following all the points.
   *
   * @return Whether the robot has finished following all the points.
   */
  public boolean isFinished();
}
