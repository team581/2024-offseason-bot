package frc.robot.intake_assist;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class IntakeAssistManager extends LifecycleSubsystem {
  private static final String LIMELIGHT_NAME = "limelight-note";
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private final ImuSubsystem imu;
  private static final double MAX_ANGLE_CHANGE = -35.0;
  private static final double MIN_ANGLE_CHANGE = 35.0;

  public IntakeAssistManager(
      LocalizationSubsystem localization, SwerveSubsystem swerve, ImuSubsystem imu) {
    super(SubsystemPriority.INTAKE_ASSIST_MANAGER);

    this.localization = localization;
    this.swerve = swerve;
    this.imu = imu;
  }



  public double getRobotRelativeAssistSpeeds(ChassisSpeeds fieldRelativeInputSpeeds ){

    double tx = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME).getEntry("tx").getDouble(0);

    Rotation2d robotToNoteAngle = Rotation2d.fromDegrees(tx);

    Rotation2d fieldRelativeNoteAngle = imu.getRobotHeading().plus(robotToNoteAngle);

    Rotation2d driverRequest = fieldRelativeNoteAngle;

    double angleError = imu.getRobotHeading().minus(driverRequest).getDegrees();

    double angleChange = Math.max(MAX_ANGLE_CHANGE, Math.min(MIN_ANGLE_CHANGE, angleError * 0.2));

    return angleChange;



  }


}
