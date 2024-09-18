package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;

public class Limelight{
  private Optional<VisionResult> rawVisionResult;
  private Optional<VisionResult> processedVisionResult;
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
}
