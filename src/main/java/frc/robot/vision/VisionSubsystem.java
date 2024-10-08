package frc.robot.vision;

import frc.robot.imu.ImuSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import java.util.Optional;

public class VisionSubsystem extends StateMachine<VisionState> {
  private final ImuSubsystem imu;
  private Optional<VisionResult> rawVisionResult;
  private Optional<VisionResult> processedVisionResult;

  public VisionSubsystem(ImuSubsystem imu) {
    super(SubsystemPriority.VISION, VisionState.DEFAULT_STATE);
    this.imu = imu;
  }

  private Optional<VisionResult> getRawVisionResult() {
    var estimatePose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");

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

  @Override
  protected void collectInputs() {
    rawVisionResult = getRawVisionResult();
    if (rawVisionResult.isEmpty()) {
      processedVisionResult = Optional.empty();
    } else {
      var rawData = rawVisionResult.get();
      processedVisionResult =
          Optional.of(
              new VisionResult(VisionUtil.interpolatePose(rawData.pose()), rawData.timestamp()));
    }
  }

  public Optional<VisionResult> getVisionResult() {
    return processedVisionResult;
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
  }
}
