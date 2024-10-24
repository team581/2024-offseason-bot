package frc.robot.localization;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.VisionResult;
import frc.robot.vision.VisionSubsystem;
import java.util.ArrayList;
import java.util.List;

public class LocalizationSubsystem extends StateMachine<LocalizationState> {
  private final ImuSubsystem imu;
  private final VisionSubsystem vision;
  private final SwerveSubsystem swerve;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final TimeInterpolatableBuffer<Pose2d> poseHistory =
      TimeInterpolatableBuffer.createBuffer(1.5);
  private double lastAddedVisionTimestamp = 0;
  private List<VisionResult> latestResult = new ArrayList<>();
  private List<VisionResult> latestInterpolatedResult = new ArrayList<>();

  public LocalizationSubsystem(ImuSubsystem imu, VisionSubsystem vision, SwerveSubsystem swerve) {
    super(SubsystemPriority.LOCALIZATION, LocalizationState.DEFAULT_STATE);
    this.swerve = swerve;
    this.imu = imu;
    this.vision = vision;
    poseEstimator =
        new SwerveDrivePoseEstimator(
            SwerveSubsystem.KINEMATICS,
            Rotation2d.fromDegrees(imu.getRobotHeading()),
            swerve.getModulePositions().toArray(new SwerveModulePosition[4]),
            new Pose2d());
  }

  @Override
  protected void collectInputs() {
    latestResult = vision.getInterpolatedVisionResult();
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    for (var results : latestResult) {
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

      poseHistory.addSample(Timer.getFPGATimestamp(), poseEstimator.getEstimatedPosition());
    }
  }

  private void resetGyro(double gyroAngle) {
    Pose2d estimatedPose =
        new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(gyroAngle));
    resetPose(estimatedPose);
  }

  public void resetPose(Pose2d estimatedPose) {
    imu.setAngle(estimatedPose.getRotation().getDegrees());
    poseEstimator.resetPosition(
        estimatedPose.getRotation(),
        swerve.getModulePositions().toArray(new SwerveModulePosition[4]),
        estimatedPose);
  }

  public Command getZeroCommand() {
    return Commands.runOnce(() -> resetGyro(FmsSubsystem.isRedAlliance() ? 0 : 180));
  }

  /**
   * Get the field relative angle the robot should face in order to be looking directly at the
   * target pose.
   */
  public double getFieldRelativeAngleToPose(Pose2d target) {
    return 0;
  }
}
