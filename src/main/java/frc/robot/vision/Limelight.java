package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.vision.interpolation.InterpolatedVision;
import java.util.Optional;

public class Limelight {
  public String limeLightName = "";

  public Optional<VisionResult> getRawVisionResult() {
    var estimatePose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLightName);

    if (estimatePose == null) {
      return Optional.empty();
    }

    if (estimatePose.tagCount == 0) {
      return Optional.empty();
    }

    // This prevents pose estimator from having crazy poses if the Limelight loses power
    if (estimatePose.pose.getX() == 0.0 && estimatePose.pose.getY() == 0.0) {
      return Optional.empty();
    }

    return Optional.of(new VisionResult(estimatePose.pose, estimatePose.timestampSeconds));
  }

  public Optional<VisionResult> getInterpolatedVisionResult() {
    var estimatePose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLightName);

    if (estimatePose == null) {
      return Optional.empty();
    }

    if (estimatePose.tagCount == 0) {
      return Optional.empty();
    }

    // This prevents pose estimator from having crazy poses if the Limelight loses power
    if (estimatePose.pose.getX() == 0.0 && estimatePose.pose.getY() == 0.0) {
      return Optional.empty();
    }
    Pose2d interpolatedPose = InterpolatedVision.leftInterpolatePose(estimatePose.pose);
    return Optional.of(new VisionResult(interpolatedPose, estimatePose.timestampSeconds));
  }
}
