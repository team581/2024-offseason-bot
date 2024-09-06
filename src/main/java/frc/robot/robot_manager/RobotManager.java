package frc.robot.robot_manager;

import frc.robot.arm.ArmState;
import frc.robot.arm.ArmSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.queuer.QueuerState;
import frc.robot.queuer.QueuerSubsystem;
import frc.robot.shooter.ShooterState;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.VisionSubsystem;

public class RobotManager extends StateMachine<RobotState> {
  public final ArmSubsystem arm;
  public final ShooterSubsystem shooter;
  public final LocalizationSubsystem localization;
  public final VisionSubsystem vision;
  private final ImuSubsystem imu;
  public final IntakeSubsystem intake;
  public final QueuerSubsystem queuer;

  private final double distanceToFeedSpot = 0;
  private final double distanceToSpeaker = 0;

  public RobotManager(
      ArmSubsystem arm,
      ShooterSubsystem shooter,
      LocalizationSubsystem localization,
      VisionSubsystem vision,
      ImuSubsystem imu,
      IntakeSubsystem intake,
      QueuerSubsystem queuer) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE_NO_GP);
    this.arm = arm;
    this.shooter = shooter;
    this.localization = localization;
    this.vision = vision;
    this.imu = imu;
    this.intake = intake;
    this.queuer = queuer;
  }

  @Override
  protected void collectInputs() {}

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    // state transition
    switch (getState()) {
      case SPEAKER_PREPARE_TO_SCORE -> {
        if (shooter.atGoal()) {
          setStateFromRequest(RobotState.SPEAKER_SCORING);
        }
      }
    }
    // on state change
    switch (getState()) {
      case SPEAKER_PREPARE_TO_SCORE -> {
        arm.setState(ArmState.SPEAKER_SHOT);
        shooter.setState(ShooterState.SPEAKER_SHOT);
        intake.setState(IntakeState.INTAKING);
        queuer.seState(QueuerState.SHOOTING);
      }
    }
    // continous sate action
    switch (getState()) {
      case SPEAKER_PREPARE_TO_SCORE -> {
        shooter.setDistanceToFeedSpot(distanceToFeedSpot);
        shooter.setDistanceToSpeaker(distanceToSpeaker);
      }
    }
  }
}
