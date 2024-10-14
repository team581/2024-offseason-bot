package frc.robot.note_assist;

import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class NoteAssistManager extends LifecycleSubsystem {
  private static final String LIMELIGHT_NAME = "limelight-note";
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private final ImuSubsystem imu;

  public NoteAssistManager(
      LocalizationSubsystem localization, SwerveSubsystem swerve, ImuSubsystem imu) {
    super(SubsystemPriority.NOTE_ASSIST_MANAGER);

    this.localization = localization;
    this.swerve = swerve;
    this.imu = imu;
  }
}
