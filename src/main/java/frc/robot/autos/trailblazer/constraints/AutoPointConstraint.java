package frc.robot.autos.trailblazer.constraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autos.trailblazer.AutoPoint;
import frc.robot.autos.trailblazer.followers.PathFollower;
import frc.robot.autos.trailblazer.trackers.PathTracker;
import java.util.List;

/** A constraint that is used while following an {@link AutoPoint}. */
public interface AutoPointConstraint {
  public static Pose2d applyPoseConstraints(Pose2d input, List<AutoPointConstraint> constraints) {
    var output = input;

    for (var constraint : constraints) {
      output = constraint.transformTargetPose(output);
    }

    return output;
  }

  public static ChassisSpeeds applyMotionConstraints(
      ChassisSpeeds input, List<AutoPointConstraint> constraints) {
    var output = input;

    for (var constraint : constraints) {
      output = constraint.transformVelocityGoal(output);
    }

    return output;
  }

  /**
   * Constrains the target pose to path follow to. Default implementation is to not change anything.
   *
   * @param input The output of the {@link PathTracker}.
   * @return The constrained target pose, used as input for the {@link PathFollower}.
   */
  public default Pose2d transformTargetPose(Pose2d input) {
    return input;
  }

  /**
   * Constrains the velocity goal to path follow to. Default implementation is to not change
   * anything.
   *
   * @param input The output of the {@link PathFollower}.
   * @return The constrained velocity goal, used as the setpoints for swerve.
   */
  public default ChassisSpeeds transformVelocityGoal(ChassisSpeeds input) {
    return input;
  }
}
