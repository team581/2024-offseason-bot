package frc.robot.localization;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.config.RobotConfig;
import frc.robot.imu.ImuSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import frc.robot.vision.VisionSubsystem;

public class LocalizationSubsystem extends StateMachine<LocalizationState> {
  private final ImuSubsystem imu;
  private final VisionSubsystem vision;
  private final SwerveDrivePoseEstimator poseEstimator;
  private double lastAddedVisionTimestamp = 0;

  public LocalizationSubsystem(ImuSubsystem imu, VisionSubsystem vision) {
    super(SubsystemPriority.LOCALIZATION, LocalizationState.DEFAULT_STATE);
    this.imu = imu;
    this.vision = vision;
    poseEstimator = new SwerveDrivePoseEstimator(null, imu.getRobotHeading(), null, new Pose2d());
  }

  @Override
  protected void collectInputs() {
    var maybeResults = vision.getVisionResult();
    if (maybeResults.isPresent()) {
      var results = maybeResults.get();
      Pose2d visionPose = results.pose();

      double visionTimestamp = results.timestamp();

      if (visionTimestamp == lastAddedVisionTimestamp) {
        // Don't add the same vision pose over and over
      } else {
        poseEstimator.addVisionMeasurement(
            visionPose,
            visionTimestamp,
            VecBuilder.fill(
                RobotConfig.get().vision().xyStdDev(),
                RobotConfig.get().vision().xyStdDev(),
                RobotConfig.get().vision().thetaStdDev()));
        lastAddedVisionTimestamp = visionTimestamp;
      }
    }
    poseHistory.addSample(Timer.getFPGATimestamp(), poseEstimator.getEstimatedPosition());
    PoseEstimate mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
    vision.setRobotPose(getPose());
  }

  public Pose2d getPose() {
    return poseEstimator;
  }
}
