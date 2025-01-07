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

  public static ChassisSpeeds constrainLinearVelocity(
      ChassisSpeeds inputSpeeds, AutoConstraintOptions options) {
    double currentLinearVelocity =
        Math.hypot(inputSpeeds.vxMetersPerSecond, inputSpeeds.vyMetersPerSecond);
    // double preserveTheta = Math.atan(inputSpeeds.vyMetersPerSecond /
    // inputSpeeds.vxMetersPerSecond);
    if (currentLinearVelocity > options.maxLinearVelocity()) {
      double clampingFactor = options.maxLinearVelocity() / currentLinearVelocity;

      return new ChassisSpeeds(
          inputSpeeds.vxMetersPerSecond * clampingFactor,
          inputSpeeds.vyMetersPerSecond * clampingFactor,
          inputSpeeds.omegaRadiansPerSecond);
    }
    return inputSpeeds;
  }

  private static ChassisSpeeds constrainRotationalVelocity(
      ChassisSpeeds inputSpeeds, AutoConstraintOptions options) {
    double currentAngularVelocity = inputSpeeds.omegaRadiansPerSecond;
    if (currentAngularVelocity > options.maxAngularVelocity()) {
      double clampingFactor = options.maxAngularVelocity() / currentAngularVelocity;
      return new ChassisSpeeds(
          inputSpeeds.vxMetersPerSecond,
          inputSpeeds.vyMetersPerSecond,
          inputSpeeds.omegaRadiansPerSecond * clampingFactor);
    }

    return inputSpeeds;
  }

  private static ChassisSpeeds constrainLinearAcceleration(
      ChassisSpeeds inputSpeeds,
      ChassisSpeeds previousSpeeds,
      double timeBetweenPreviousAndInputSpeeds,
      AutoConstraintOptions options) {


    double currentLinearAcceleration =
        Math.hypot(inputSpeeds.vxMetersPerSecond, inputSpeeds.vyMetersPerSecond);
    double previousLinearAcceleration =
        Math.hypot(previousSpeeds.vxMetersPerSecond, previousSpeeds.vyMetersPerSecond);
    double unconstrainedLinearAcceleration =
        (currentLinearAcceleration - previousLinearAcceleration)
            / timeBetweenPreviousAndInputSpeeds;
    double preserveTheta = Math.atan(inputSpeeds.vyMetersPerSecond / inputSpeeds.vxMetersPerSecond);
    if (unconstrainedLinearAcceleration > options.maxLinearAcceleration()) {
      double finalAcceleration =
          previousLinearAcceleration
              + options.maxLinearAcceleration() * timeBetweenPreviousAndInputSpeeds;
      double constrainedVx = finalAcceleration * Math.cos(preserveTheta);
      double constrainedVy = finalAcceleration * Math.sin(preserveTheta);
      return new ChassisSpeeds(constrainedVx, constrainedVy, inputSpeeds.omegaRadiansPerSecond);
    }
    return inputSpeeds;
  }

  private static ChassisSpeeds constrainRotationalAcceleration(
      ChassisSpeeds inputSpeeds,
      ChassisSpeeds previousSpeeds,
      double timeBetweenPreviousAndInputSpeeds,
      AutoConstraintOptions options) {

    double currentAngularSpeed = inputSpeeds.omegaRadiansPerSecond;
    double previousAngularSpeed = previousSpeeds.omegaRadiansPerSecond;

    double currentAngularAcceleration =
        currentAngularSpeed - previousAngularSpeed / timeBetweenPreviousAndInputSpeeds;
    if (currentAngularAcceleration > options.maxAngularAcceleration()) {
      double constrainedAngularAcceleration =
          previousAngularSpeed
              + options.maxAngularAcceleration() * timeBetweenPreviousAndInputSpeeds;
      return new ChassisSpeeds(
          inputSpeeds.vxMetersPerSecond,
          inputSpeeds.vyMetersPerSecond,
          constrainedAngularAcceleration);
    }
    return inputSpeeds;
  }

  private AutoConstraintCalculator() {}
}
