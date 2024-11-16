package frc.robot.autos.trailblazer.constraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// TODO: To consume this, transform target point then apply velocity transform
public interface AutoPointConstraint {
  // TODO: Probably get rid of transforming target pose
  public default Pose2d transformTargetPose(Pose2d input) {
    return input;
  }

  public default ChassisSpeeds transformVelocityGoal(ChassisSpeeds input) {
    return input;
  }
}
