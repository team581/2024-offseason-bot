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
  public ChassisSpeeds transformVelocityGoal(ChassisSpeeds input) {
    // TODO: Implement https://github.com/team581/2024-offseason-bot/issues/96
    return input;
  }
}
