package frc.robot.autos.trailblazer.constraints;

import edu.wpi.first.math.geometry.Pose2d;

// TODO: Implement this for funsies if we want collision avoidance
public class CollisionAvoidanceConstraint implements AutoPointConstraint {
  @Override
  public Pose2d transformTargetPose(Pose2d input) {
    return input;
  }
}
