package frc.robot.autos.trailblazer.constraints;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoAccelerationConstraint implements AutoPointConstraint {
  private final double maxLinearAcceleration;
  private final double maxAngularAcceleration;

  public AutoAccelerationConstraint(double maxLinearAcceleration, double maxAngularAcceleration) {
    this.maxLinearAcceleration = maxLinearAcceleration;
    this.maxAngularAcceleration = maxAngularAcceleration;
  }

  @Override
  public ChassisSpeeds transformVelocityGoal(ChassisSpeeds input) {
    // TODO: Implement https://github.com/team581/2024-offseason-bot/issues/97
    return input;
  }
}
