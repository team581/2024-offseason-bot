package frc.robot.autos.trailblazer.constraints;

import edu.wpi.first.math.geometry.Pose2d;

public class AutoAccelerationConstraint {
  private final double maxLinearAcceleration;
  private final double maxAngularAcceleration;

  public AutoAccelerationConstraint(double maxLinearAcceleration, double maxAngularAcceleration) {
    this.maxLinearAcceleration = maxLinearAcceleration;
    this.maxAngularAcceleration = maxAngularAcceleration;
  }

  public Pose2d transform(Pose2d input) {
    // TODO: Implement
    return input;
  }
}
