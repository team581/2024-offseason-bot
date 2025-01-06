package frc.robot.autos.trailblazer.constraints;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoConstraintCalculator {

  public static AutoConstraintOptions generateLinearVelocityConstraint(
      Pose2d robotPose, SegmentMotionProfile motionProfile, AutoConstraintOptions constraints) {
    var endPoint = motionProfile.endPose();
    var endVelocity = motionProfile.endVelocity();
    var currentVelocity = motionProfile.currentVelocity();
    var maxAcceleration = motionProfile.maxLinearAcceleration();
    DogLog.log("Trailblazer/Constraints/VelocityCalculation/EndVelocityGoal", endVelocity);
    DogLog.log("Trailblazer/Constraints/VelocityCalculation/CurrentVelocity", currentVelocity);
    DogLog.log("Trailblazer/Constraints/VelocityCalculation/MaxAccelaration", maxAcceleration);

    var distanceToEnd = robotPose.getTranslation().getDistance(endPoint.getTranslation());
    var currentAcceleration =
        ((endVelocity * endVelocity) - (currentVelocity * currentVelocity)) / (2 * distanceToEnd);
    if (Math.abs(currentAcceleration) > maxAcceleration) {
      var goalAcceleration = Math.copySign(maxAcceleration, currentAcceleration);
      var newVelocity =
          Math.sqrt((endVelocity * endVelocity) - (goalAcceleration * (2 * distanceToEnd)));

      return new AutoConstraintOptions(
          constraints.collisionAvoidance(),
          newVelocity,
          constraints.maxAngularVelocity(),
          constraints.maxLinearAcceleration(),
          constraints.maxAngularAcceleration());
    }
    return constraints;
  }

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

    double deltaVx = inputSpeeds.vxMetersPerSecond - previousSpeeds.vxMetersPerSecond;
    double deltaVy = inputSpeeds.vyMetersPerSecond - previousSpeeds.vyMetersPerSecond;
    double unconstrainedLinearAcceleration =
        Math.sqrt(deltaVx * deltaVx + deltaVy * deltaVy) / timeBetweenPreviousAndInputSpeeds;

    double constrainedLinearAcceleration =
        Math.min(unconstrainedLinearAcceleration, options.maxLinearAcceleration());

    if (unconstrainedLinearAcceleration > options.maxLinearAcceleration()) {
      double constrainedVx =
          previousSpeeds.vxMetersPerSecond
              + (deltaVx / unconstrainedLinearAcceleration) * constrainedLinearAcceleration;
      double constrainedVy =
          previousSpeeds.vyMetersPerSecond
              + (deltaVy / unconstrainedLinearAcceleration) * constrainedLinearAcceleration;

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
