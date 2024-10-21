package frc.robot.intake_assist;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.LimelightHelpers;

public class IntakeAssistManager extends LifecycleSubsystem {
  private static final double ASSIST_KP = 0.2;
  private static final String LIMELIGHT_NAME = "limelight-note";
  private final SwerveSubsystem swerve;
  private final ImuSubsystem imu;
  private static final double MAX_ANGLE_CHANGE = -35.0;
  private static final double MIN_ANGLE_CHANGE = 35.0;

  public IntakeAssistManager(SwerveSubsystem swerve, ImuSubsystem imu) {
    super(SubsystemPriority.INTAKE_ASSIST_MANAGER);
    this.swerve = swerve;
    this.imu = imu;
  }



  public ChassisSpeeds getRobotRelativeAssistSpeeds(ChassisSpeeds fieldRelativeInputSpeeds ){

    double tx = LimelightHelpers.getTX(LIMELIGHT_NAME);


    if (tx == 0) {
      return fieldRelativeInputSpeeds;
    }

    DogLog.log("IntakeAssist/TX", tx);

    double fieldRelativeNoteAngle = imu.getRobotHeading()  + tx;

    double angleError = imu.getRobotHeading() - fieldRelativeNoteAngle;

    double angleChange = MathUtil.clamp(MIN_ANGLE_CHANGE, MAX_ANGLE_CHANGE, angleError * ASSIST_KP);

    Translation2d requestedFieldRelativeDrive = new Translation2d(fieldRelativeInputSpeeds.vxMetersPerSecond,
    fieldRelativeInputSpeeds.vyMetersPerSecond);

    Translation2d newDriveRequest = requestedFieldRelativeDrive.rotateBy(Rotation2d.fromDegrees(angleChange));

    return new ChassisSpeeds(newDriveRequest.getX(), newDriveRequest.getY(), fieldRelativeInputSpeeds.omegaRadiansPerSecond);

  }

  public void updateSwerveSpeeds() {
    ChassisSpeeds intakeAssistSpeeds = getRobotRelativeAssistSpeeds(swerve.getFieldRelativeSpeeds());
    swerve.setIntakeAssistTeleopSpeeds(intakeAssistSpeeds);
    swerve.setIntakeAssistAutoSpeeds(intakeAssistSpeeds);
}

}
