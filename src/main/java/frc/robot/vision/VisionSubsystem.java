package frc.robot.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.imu.ImuSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import java.util.ArrayList;
import java.util.List;

public class VisionSubsystem extends StateMachine<VisionState> {
  private final ImuSubsystem imu;
  private final Limelight leftLimelight;
  private final Limelight rightLimelight;
  private final List<VisionResult> interpolatedVisionResult = new ArrayList<>();
  private double robotHeading;
  private double pitch;
  private double angularVelocity;
  private double pitchRate;
  private double roll;
  private double rollRate;

  // right is positive x, up is positive y, forward is positive z
  private final Pose3d robotPoseCalibrationTargetSpaceMetersRadians = new Pose3d(0, -1.0, -1.5, new Rotation3d(0.0, 0.0, 0.0));

  public static final Pose2d ORIGINAL_RED_SPEAKER =
      new Pose2d(
          Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(180));
  public static final Pose2d ORIGINAL_BLUE_SPEAKER =
      new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0));

  public static final Pose2d RED_FEED_SPOT_AMP_AREA =
      new Pose2d(15.9, 7.5, Rotation2d.fromDegrees(180));
  public static final Pose2d BLUE_FEED_SPOT_AMP_AREA =
      new Pose2d(0.6, 7.5, Rotation2d.fromDegrees(0));

  public VisionSubsystem(ImuSubsystem imu, Limelight leftLimelight, Limelight rightLimelight) {
    super(SubsystemPriority.VISION, VisionState.DEFAULT_STATE);
    this.imu = imu;
    this.leftLimelight = leftLimelight;
    this.rightLimelight = rightLimelight;
  }

  @Override
  protected void collectInputs() {
    robotHeading = imu.getRobotHeading();
    DogLog.log("Vision/RobotHeading", imu.getRobotHeading());
    angularVelocity = imu.getRobotAngularVelocity();
    pitch = imu.getPitch();
    pitchRate = imu.getPitchRate();
    roll = imu.getRoll();
    rollRate = imu.getRollRate();

    var leftInterpolatedVisionResult = leftLimelight.getInterpolatedVisionResult();
    var rightInterpolatedVisionResult = rightLimelight.getInterpolatedVisionResult();

    interpolatedVisionResult.clear();

    if (!DriverStation.isAutonomous()) {
      if (leftInterpolatedVisionResult.isPresent()) {
        interpolatedVisionResult.add(leftInterpolatedVisionResult.get());
      }
    }
    if (rightInterpolatedVisionResult.isPresent()) {
      interpolatedVisionResult.add(rightInterpolatedVisionResult.get());
    }
  }

  public List<VisionResult> getInterpolatedVisionResult() {
    return interpolatedVisionResult;
  }

  private void logCalibrationValues() {
    var leftCameraPoseTargetSpace = LimelightHelpers.getCameraPose3d_TargetSpace("limelight");
    var rightCameraPoseTargetSpace = LimelightHelpers.getCameraPose3d_TargetSpace("limelight");

    var leftCameraRobotRelativePose = getRobotRelativeCameraOffset(robotPoseCalibrationTargetSpaceMetersRadians,
        leftCameraPoseTargetSpace);
    var rightCameraRobotRelativePose = getRobotRelativeCameraOffset(robotPoseCalibrationTargetSpaceMetersRadians, rightCameraPoseTargetSpace);

    DogLog.log("Calibration/LeftCamera/Right", leftCameraRobotRelativePose.getX());
    DogLog.log("Calibration/LeftCamera/Up", leftCameraRobotRelativePose.getY());
    DogLog.log("Calibration/LeftCamera/Forward", leftCameraRobotRelativePose.getZ());
    DogLog.log("Calibration/LeftCamera/Roll", leftCameraRobotRelativePose.getRotation().getX());
    DogLog.log("Calibration/LeftCamera/Pitch", leftCameraRobotRelativePose.getRotation().getY());
    DogLog.log("Calibration/LeftCamera/Yaw", leftCameraRobotRelativePose.getRotation().getZ());

    DogLog.log("Calibration/RightCamera/Right", rightCameraRobotRelativePose.getX());
    DogLog.log("Calibration/RightCamera/Up", rightCameraRobotRelativePose.getY());
    DogLog.log("Calibration/RightCamera/Forward", rightCameraRobotRelativePose.getZ());
    DogLog.log("Calibration/RightCamera/Roll", rightCameraRobotRelativePose.getRotation().getX());
    DogLog.log("Calibration/RightCamera/Pitch", rightCameraRobotRelativePose.getRotation().getY());
    DogLog.log("Calibration/RightCamera/Yaw", rightCameraRobotRelativePose.getRotation().getZ());
  }

  private Pose3d getRobotRelativeCameraOffset(Pose3d robotPoseTargetSpace, Pose3d seenCameraPoseTargetSpace) {
      // Positive X = Right
      var cameraLeftRight = seenCameraPoseTargetSpace.getX();
      // Positive Y = Down, so flipped for common sense
      var cameraUpDown = -1 * (seenCameraPoseTargetSpace.getY());
      // Positive Z = Backward, so flipped for common sense
      var cameraForwardBackward = -1 * (seenCameraPoseTargetSpace.getZ());
      // Pitch rotates around left right axis (x according to LL coordinate systems)
      var cameraPitch = Units.degreesToRadians(seenCameraPoseTargetSpace.getRotation().getX());
      // Roll rotates around forward backward axis (Z according to LL coordinate systems)
      var cameraRoll = Units.degreesToRadians(seenCameraPoseTargetSpace.getRotation().getZ());
      // Yaw rotates around up down axis (y according to LL coordinate systems)
      var cameraYaw = Units.degreesToRadians(seenCameraPoseTargetSpace.getRotation().getY());

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

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    rightLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    leftLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    DogLog.log("Vision/AngularVelocity", angularVelocity);
    DogLog.log("Vision/Pitch", pitch);
    DogLog.log("Vision/visionIsEmpty", getInterpolatedVisionResult().isEmpty());

    DogLog.log("Vision/CombinedVisionState", getVisionState());
    DogLog.log("Vision/Left/VisionState", leftLimelight.getState());
    DogLog.log("Vision/Right/VisionState", rightLimelight.getState());

    if (RobotConfig.IS_CALIBRATION) {
      logCalibrationValues();
    }
  }

  public CameraStatus getVisionState() {
    var leftState = leftLimelight.getState();
    var rightState = rightLimelight.getState();
    if (leftState == CameraStatus.OFFLINE && rightState == CameraStatus.OFFLINE) {
      return CameraStatus.OFFLINE;
    }

    if (leftState == CameraStatus.SEES_TAGS || rightState == CameraStatus.SEES_TAGS) {
      return CameraStatus.SEES_TAGS;
    }

    return CameraStatus.ONLINE_NO_TAGS;
  }

  /** Same as the regular vision state but returns OFFLINE if any camera is offline. */
  public CameraStatus getPessimisticVisionState() {
    var leftState = leftLimelight.getState();
    var rightState = rightLimelight.getState();

    if (leftState == CameraStatus.OFFLINE || rightState == CameraStatus.OFFLINE) {
      return CameraStatus.OFFLINE;
    }

    if (leftState == CameraStatus.SEES_TAGS || rightState == CameraStatus.SEES_TAGS) {
      return CameraStatus.SEES_TAGS;
    }

    return CameraStatus.ONLINE_NO_TAGS;
  }
}
