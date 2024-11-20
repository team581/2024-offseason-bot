package frc.robot.autos.trailblazer;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.trailblazer.constraints.AutoConstraintCalculator;
import frc.robot.autos.trailblazer.constraints.AutoConstraintOptions;
import frc.robot.autos.trailblazer.followers.PathFollower;
import frc.robot.autos.trailblazer.followers.PidPathFollower;
import frc.robot.autos.trailblazer.trackers.HeuristicPathTracker;
import frc.robot.autos.trailblazer.trackers.PathTracker;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;

public class Trailblazer {
  /**
   * Given a point and the constraints for its parent segment, resolve the constraint options to use
   * while following that point.
   */
  private static AutoConstraintOptions resolveConstraints(
      AutoPoint point, AutoConstraintOptions segmentConstraints) {
    return point.constraints.orElse(segmentConstraints);
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
                  pathTracker.updateRobotState(
                      localization.getPose(), swerve.getFieldRelativeSpeeds());

                  var currentAutoPoint = pathTracker.getCurrentPoint();

                  var constrainedVelocityGoal =
                      getSwerveSetpoint(currentAutoPoint, segment.defaultConstraints);
                  swerve.setFieldRelativeAutoSpeeds(constrainedVelocityGoal);

                  if (previousAutoPoint != currentAutoPoint) {
                    // Currently tracked point has changed, trigger side effects
                    currentAutoPoint.command.schedule();
                    previousAutoPoint = currentAutoPoint;
                    DogLog.timestamp("Trailblazer/Tracker/NewPoint");
                  }
                },
                swerve))
        .until(pathTracker::isFinished);
  }

  private ChassisSpeeds getSwerveSetpoint(
      AutoPoint point, AutoConstraintOptions segmentConstraints) {
    var usedConstraints = resolveConstraints(point, segmentConstraints);

    var originalTargetPose = pathTracker.getTargetPose();
    DogLog.log("Trailblazer/Tracker/RawOutput", originalTargetPose);
    var constrainedTargetPose =
        AutoConstraintCalculator.constrainTargetPose(originalTargetPose, usedConstraints);
    DogLog.log("Trailblazer/Tracker/UsedOutput", constrainedTargetPose);

    var originalVelocityGoal =
        pathFollower.calculateSpeeds(localization.getPose(), constrainedTargetPose);
    DogLog.log("Trailblazer/Follower/RawOutput", originalVelocityGoal);
    var constrainedVelocityGoal =
        AutoConstraintCalculator.constrainVelocityGoal(originalVelocityGoal, usedConstraints);
    DogLog.log("Trailblazer/Follower/UsedOutput", constrainedVelocityGoal);

    return constrainedVelocityGoal;
  }
}
