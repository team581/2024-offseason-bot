package frc.robot.autos.trailblazer.constraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoConstraintCalculator {

  public static ChassisSpeeds constrainLinearVelocity(
      ChassisSpeeds inputSpeeds,
      AutoConstraintOptions options){
    // TODO: Implement linear velocity constraint
    return inputSpeeds;
  }

  public static ChassisSpeeds constrainRotationalVelocity(
      ChassisSpeeds inputSpeeds,
      AutoConstraintOptions options) {
    // TODO: Implement rotational velocity constraint
    return inputSpeeds;
  }

  public static ChassisSpeeds constrainLinearAcceleration(
      ChassisSpeeds inputSpeeds,
      ChassisSpeeds previousSpeeds,
      double timeBetweenPreviousAndInputSpeeds,
      AutoConstraintOptions options){
    // TODO: Implement linear acceleration constraint
    return inputSpeeds;
  }

  public static ChassisSpeeds constrainRotationalAcceleration(
      ChassisSpeeds inputSpeeds,
      ChassisSpeeds previousSpeeds,
      double timeBetweenPreviousAndInputSpeeds,
      AutoConstraintOptions options){
    // TODO: Implement angular acceleration constraint
    return inputSpeeds;
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
      constrainedSpeeds = constrainLinearAcceleration(constrainedSpeeds, previousSpeeds, timeBetweenPreviousAndInputSpeeds, options);
    }

    if (options.maxAngularAcceleration() != 0) {
      constrainedSpeeds = constrainRotationalAcceleration(constrainedSpeeds, previousSpeeds, timeBetweenPreviousAndInputSpeeds, options);
    }

    return constrainedSpeeds;
  }

  private AutoConstraintCalculator() {}
}
