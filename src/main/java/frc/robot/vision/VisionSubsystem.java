package frc.robot.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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

  public static final Pose2d ORIGINAL_RED_SPEAKER =
      new Pose2d(
          Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(180));
  public static final Pose2d ORIGINAL_BLUE_SPEAKER =
      new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0));

  public static final Pose2d RED_FEED_SPOT_AMP_AREA =
      new Pose2d(9.5, 7, Rotation2d.fromDegrees(180));
  public static final Pose2d BLUE_FEED_SPOT_AMP_AREA = new Pose2d(7, 7, Rotation2d.fromDegrees(0));

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

    if (leftInterpolatedVisionResult.isPresent()) {
      interpolatedVisionResult.add(leftInterpolatedVisionResult.get());
    }
    if (rightInterpolatedVisionResult.isPresent()) {
      interpolatedVisionResult.add(rightInterpolatedVisionResult.get());
    }
  }

  public List<VisionResult> getInterpolatedVisionResult() {
    return interpolatedVisionResult;
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
  }

  public VisionState getVisionState() {
    var leftState = leftLimelight.getState();
    var rightState = rightLimelight.getState();
    if (leftState == VisionState.OFFLINE && rightState == VisionState.OFFLINE) {
      return VisionState.OFFLINE;
    } else if (leftState == VisionState.ONLINE_NO_TAGS
        && rightState == VisionState.ONLINE_NO_TAGS) {
      return VisionState.ONLINE_NO_TAGS;
    } else if (leftState == VisionState.SEES_TAGS || rightState == VisionState.SEES_TAGS) {
      return VisionState.SEES_TAGS;
    }
    return VisionState.DEFAULT_STATE;
  }
}
