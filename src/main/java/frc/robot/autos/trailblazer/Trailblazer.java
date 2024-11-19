package frc.robot.autos.trailblazer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.trailblazer.constraints.AutoPointConstraint;
import frc.robot.autos.trailblazer.followers.PathFollower;
import frc.robot.autos.trailblazer.followers.PidPathFollower;
import frc.robot.autos.trailblazer.trackers.HeuristicPathTracker;
import frc.robot.autos.trailblazer.trackers.PathTracker;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import java.util.List;

public class Trailblazer {
  /**
   * Global default constraints, which will be applied to points that have no constraints specified,
   * and no default constraints specified for the segment.
   */
  private static final List<AutoPointConstraint> globalDefaultConstraints = List.of();

  private static List<AutoPointConstraint> resolveConstraints(
      List<AutoPointConstraint> segmentConstraints, AutoPoint point) {
    if (!point.constraints.isEmpty()) {
      return point.constraints;
    }

    if (!segmentConstraints.isEmpty()) {
      return segmentConstraints;
    }

    return globalDefaultConstraints;
  }

  private final SwerveSubsystem swerve;
  private final LocalizationSubsystem localization;
  private final PathTracker pathTracker = new HeuristicPathTracker();
  private final PathFollower pathFollower =
      new PidPathFollower(
          new PIDController(0, 0, 0), new PIDController(0, 0, 0), new PIDController(0, 0, 0));
  private AutoPoint previousAutoPoint = new AutoPoint(new Pose2d());

  public Trailblazer(SwerveSubsystem swerve, LocalizationSubsystem localization) {
    this.swerve = swerve;
    this.localization = localization;
  }

  public Command followSegment(AutoSegment segment) {
    return Commands.runOnce(() -> pathTracker.resetAndSetPoints(segment.points))
        .alongWith(
            Commands.run(
                () -> {
                  var currentAutoPoint = pathTracker.getCurrentPoint();
                  var constrainedVelocityGoal =
                      getSwerveSetpoint(currentAutoPoint, segment.defaultConstraints);
                  swerve.setFieldRelativeAutoSpeeds(constrainedVelocityGoal);

                  if (previousAutoPoint != currentAutoPoint) {
                    // Currently tracked point has changed, trigger side effects
                    currentAutoPoint.command.schedule();
                    previousAutoPoint = currentAutoPoint;
                  }
                },
                swerve));
  }

  private ChassisSpeeds getSwerveSetpoint(
      AutoPoint point, List<AutoPointConstraint> segmentConstraints) {
    var usedConstraints = resolveConstraints(segmentConstraints, point);

    var currentPose = localization.getPose();
    var fieldRelativeRobotSpeeds = swerve.getFieldRelativeSpeeds();
    var originalTargetPose = pathTracker.getTargetPose(currentPose, fieldRelativeRobotSpeeds);
    var constrainedTargetPose =
        AutoPointConstraint.applyPoseConstraints(originalTargetPose, usedConstraints);

    var originalVelocityGoal = pathFollower.calculateSpeeds(currentPose, constrainedTargetPose);
    return AutoPointConstraint.applyMotionConstraints(originalVelocityGoal, usedConstraints);
  }
}
