package frc.robot.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.RobotConfig;
import frc.robot.vision.interpolation.CameraDataset;
import frc.robot.vision.interpolation.InterpolatedVision;
import java.util.Optional;

public class Limelight {
  private final String limelightTableName;
  public final String name;
  private CameraDataset interpolationData;
  private CameraStatus state = CameraStatus.ONLINE_NO_TAGS;
  private double limelightHeartbeat = -1;

  private final Timer limelightTimer = new Timer();

  public Limelight(String name, CameraDataset interpolationData) {
    limelightTableName = "limelight-" + name;
    this.name = name;
    this.interpolationData = interpolationData;
    limelightTimer.start();
  }

  public void sendImuData(
      double robotHeading,
      double angularVelocity,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate) {
    LimelightHelpers.SetRobotOrientation(
        limelightTableName, robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
  }

  public Optional<VisionResult> getInterpolatedVisionResult() {
    var rawResult = getRawVisionResult();

    updateState(rawResult);

    if (rawResult.isEmpty()) {
      return Optional.empty();
    }

    Pose2d interpolatedPose =
        InterpolatedVision.interpolatePose(rawResult.get().pose(), interpolationData);
    return Optional.of(new VisionResult(interpolatedPose, rawResult.get().timestamp()));
  }

  public void logCameraPositionCalibrationValues() {
    var cameraPoseTargetSpace = LimelightHelpers.getCameraPose3d_TargetSpace(limelightTableName);
    var robotPoseTargetSpace = RobotConfig.get().vision().robotPoseCalibrationTargetSpace();
    var leftCameraRobotRelativePose =
        getRobotRelativeCameraPosition(robotPoseTargetSpace, cameraPoseTargetSpace);
    DogLog.log(
        "CameraPositionCalibration/" + name + "/LL Right", leftCameraRobotRelativePose.getX());
    DogLog.log("CameraPositionCalibration/" + name + "/LL Up", leftCameraRobotRelativePose.getY());
    DogLog.log(
        "CameraPositionCalibration/" + name + "/LL Forward", leftCameraRobotRelativePose.getZ());
    DogLog.log(
        "CameraPositionCalibration/" + name + "/LL Roll",
        Units.radiansToDegrees(leftCameraRobotRelativePose.getRotation().getX()));
    DogLog.log(
        "CameraPositionCalibration/" + name + "/LL Pitch",
        Units.radiansToDegrees(leftCameraRobotRelativePose.getRotation().getY()));
    DogLog.log(
        "CameraPositionCalibration/" + name + "/LL Yaw",
        Units.radiansToDegrees(leftCameraRobotRelativePose.getRotation().getZ()));
  }

  private Pose3d getRobotRelativeCameraPosition(
      Pose3d robotPoseTargetSpace, Pose3d seenCameraPoseTargetSpace) {
    // Positive X = Right
    var cameraLeftRight = seenCameraPoseTargetSpace.getX();
    // Positive Y = Down, so flipped for common sense
    var cameraUpDown = -1 * (seenCameraPoseTargetSpace.getY());
    // Positive Z = Backward, so flipped for common sense
    var cameraForwardBackward = -1 * (seenCameraPoseTargetSpace.getZ());
    // Pitch rotates around left right axis (x according to LL coordinate systems)
    var cameraPitch = seenCameraPoseTargetSpace.getRotation().getX();
    // Roll rotates around forward backward axis (Z according to LL coordinate systems)
    var cameraRoll = seenCameraPoseTargetSpace.getRotation().getZ();
    // Yaw rotates around up down axis (y according to LL coordinate systems)
    var cameraYaw = seenCameraPoseTargetSpace.getRotation().getY();

    var robotLeftRight = robotPoseTargetSpace.getX();
    var robotUpDown = robotPoseTargetSpace.getY();
    var robotForwardBackward = robotPoseTargetSpace.getZ();
    var robotPitch = robotPoseTargetSpace.getRotation().getY();
    var robotRoll = robotPoseTargetSpace.getRotation().getX();
    var robotYaw = robotPoseTargetSpace.getRotation().getZ();

    var right = cameraLeftRight - robotLeftRight;
    var up = cameraUpDown - robotUpDown;
    var forward = cameraForwardBackward - robotForwardBackward;
    var roll = cameraRoll - robotRoll;
    var pitch = cameraPitch - robotPitch;
    var yaw = cameraYaw - robotYaw;
    return new Pose3d(right, up, forward, new Rotation3d(roll, pitch, yaw));
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

  private void updateState(Optional<VisionResult> rawResult) {
    var newHeartbeat = LimelightHelpers.getLimelightNTDouble(limelightTableName, "hb");

    if (limelightHeartbeat != newHeartbeat) {
      limelightTimer.restart();
    }
    limelightHeartbeat = newHeartbeat;

    if (limelightTimer.hasElapsed(5)) {
      state = CameraStatus.OFFLINE;
      return;
    }

    if (!rawResult.isEmpty()) {
      state = CameraStatus.SEES_TAGS;
      return;
    }
    state = CameraStatus.ONLINE_NO_TAGS;
  }

  public CameraStatus getState() {
    return state;
  }
}
