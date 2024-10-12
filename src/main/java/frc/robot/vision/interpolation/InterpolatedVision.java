package frc.robot.vision.interpolation;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;

public class InterpolatedVision {
  private static final LeftInterpolatedVisionDataset usedSet =
      RobotConfig.get().vision().leftinterpolatedVisionSet();

  /**
   * @param visionInput - pose from the limelight
   * @return a transformed pose that can be added to the pose estimator
   */
  public static Pose2d leftInterpolatePose(Pose2d visionInput) {
    var usedDataPoints = FmsSubsystem.isRedAlliance() ? usedSet.redSet : usedSet.blueSet;

    return new Pose2d(
        InterpolationUtil.interpolateTranslation(usedDataPoints, visionInput.getTranslation()),
        visionInput.getRotation());
  }
  public static Pose2d rightInterpolatePose(Pose2d visionInput) {
    var usedDataPoints = FmsSubsystem.isRedAlliance() ? usedSet.redSet : usedSet.blueSet;

    return new Pose2d(
        InterpolationUtil.interpolateTranslation(usedDataPoints, visionInput.getTranslation()),
        visionInput.getRotation());
  }

  private InterpolatedVision() {}
}
