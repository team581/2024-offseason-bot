package frc.robot.autos.trailblazer.constraints;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoConstraintCalculator {

  public static ChassisSpeeds constrainVelocityGoal(
      ChassisSpeeds inputSpeeds,
      ChassisSpeeds previousSpeeds,
      double timeBetweenPreviousAndInputSpeeds,
      AutoConstraintOptions options) {
    ChassisSpeeds constrainedSpeeds = inputSpeeds;

    if (options.maxLinearVelocity() != 0) {
      constrainedSpeeds = constrainLinearVelocity(constrainedSpeeds, options);
    }

    if (options.maxAngularVelocity() != 0) {
      constrainedSpeeds = constrainRotationalVelocity(constrainedSpeeds, options);
    }

    if (options.maxLinearAcceleration() != 0) {
      constrainedSpeeds =
          constrainLinearAcceleration(
              constrainedSpeeds, previousSpeeds, timeBetweenPreviousAndInputSpeeds, options);
    }

    if (options.maxAngularAcceleration() != 0) {
      constrainedSpeeds =
          constrainRotationalAcceleration(
              constrainedSpeeds, previousSpeeds, timeBetweenPreviousAndInputSpeeds, options);
    }

    return constrainedSpeeds;
  }

  private static ChassisSpeeds constrainLinearVelocity(
      ChassisSpeeds inputSpeeds, AutoConstraintOptions options) {
    // TODO: Implement linear velocity constraint
    return inputSpeeds;
  }

  private static ChassisSpeeds constrainRotationalVelocity(
      ChassisSpeeds inputSpeeds, AutoConstraintOptions options) {
    // TODO: Implement rotational velocity constraint
    return inputSpeeds;
  }

  private static ChassisSpeeds constrainLinearAcceleration(
      ChassisSpeeds inputSpeeds,
      ChassisSpeeds previousSpeeds,
      double timeBetweenPreviousAndInputSpeeds,
      AutoConstraintOptions options) {
    // TODO: Implement linear acceleration constraint
    // Could approach this by seeing if the acceleration exceeds the max acceleration.
    // If it does, calculate the maximum velocity to achieve the max acceleration and
    // use the same velocity clamp function as above.
    return inputSpeeds;
  }

  private static ChassisSpeeds constrainRotationalAcceleration(
      ChassisSpeeds inputSpeeds,
      ChassisSpeeds previousSpeeds,
      double timeBetweenPreviousAndInputSpeeds,
      AutoConstraintOptions options) {
    // TODO: Implement angular acceleration constraint
    return inputSpeeds;
  }

  private AutoConstraintCalculator() {}
}
