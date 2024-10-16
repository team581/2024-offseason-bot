package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.vision.LimelightHelpers.LimelightResults;
import frc.robot.vision.interpolation.InterpolatedVision;
import java.util.Optional;

import dev.doglog.DogLog;

public class Limelight {
  private Optional<VisionResult> rawVisionResult;
  private Optional<VisionResult> processedVisionResult;
  private final String limelightTableName;
  private final String name;

  public Limelight(String name) {
    limelightTableName = "limelight-" + name;
    this.name = name;
  }

  public Optional<VisionResult> getInterpolatedVisionResult() {
    var rawResult = getRawVisionResult();

    if (rawResult.isEmpty()) {
      return Optional.empty();
    }
    
    Pose2d interpolatedPose = InterpolatedVision.interpolatePose(rawResult.get().pose());
    return Optional.of(new VisionResult(interpolatedPose, rawResult.get().timestamp()));
  }

  private Optional<VisionResult> getRawVisionResult() {
    var estimatePose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightTableName);

    if (estimatePose == null) {
      return Optional.empty();
    }

    DogLog.log("Vision/" + name + "/RawLimelightPose", estimatePose.pose);

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
