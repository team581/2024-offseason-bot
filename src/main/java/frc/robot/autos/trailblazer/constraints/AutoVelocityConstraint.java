package frc.robot.autos.trailblazer.constraints;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoVelocityConstraint implements AutoPointConstraint {
  private final double maxLinearVelocity;
  private final double maxAngularVelocity;

  public AutoVelocityConstraint(double maxLinearVelocity, double maxAngularVelocity) {
    this.maxLinearVelocity = maxLinearVelocity;
    this.maxAngularVelocity = maxAngularVelocity;
  }

  @Override
  public ChassisSpeeds transform(ChassisSpeeds input) {
    // TODO: Implement
    return input;
  }
}
