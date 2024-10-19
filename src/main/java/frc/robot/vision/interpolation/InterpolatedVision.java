package frc.robot.vision.interpolation;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;

public class InterpolatedVision {
  private static final InterpolatedVisionDataset dataSet =
      RobotConfig.get().vision().interpolatedVisionSet();

  /**
   * @param visionInput - pose from the limelight
   * @return a transformed pose that can be added to the pose estimator
   */
  public static Pose2d leftInterpolatePose(Pose2d visionInput) {
    return interpolatePose(visionInput, dataSet.leftSet);
  }

  public static Pose2d rightInterpolatePose(Pose2d visionInput) {
    return interpolatePose(visionInput, dataSet.rightSet);
  }

  private static Pose2d interpolatePose(Pose2d visionInput, CameraDataset dataset) {
    var usedDataPoints = FmsSubsystem.isRedAlliance() ? dataset.red() : dataset.blue();

    return new Pose2d(
        InterpolationUtil.interpolateTranslation(usedDataPoints, visionInput.getTranslation()),
        visionInput.getRotation());
  }

  private InterpolatedVision() {}
}
