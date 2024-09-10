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
  public final ImuSubsystem imu;
  public final IntakeSubsystem intake;
  public final QueuerSubsystem queuer;

  private final double distanceToFeedSpot = 0.0;
  private final double distanceToSpeaker = 0.0;

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
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case SPEAKER_WAITING,
              AMP_WAITING,
              FEEDING_WAITING,
              PASS_WAITING,
              IDLE_NO_GP,
              IDLE_WITH_GP,
              CLIMBING_1_LINEUP,
              CLIMBING_2_HANGING ->
          currentState;
      case SPEAKER_SCORING, AMP_SCORING, FEEDING_SHOOTING, PASS_SHOOTING ->
          queuer.hasNote() ? currentState : RobotState.IDLE_NO_GP;

      case SPEAKER_PREPARE_TO_SCORE ->
          shooter.atGoal() && arm.atGoal() ? RobotState.SPEAKER_SCORING : currentState;

      case AMP_PREPARE_TO_SCORE ->
          shooter.atGoal() && arm.atGoal() ? RobotState.AMP_SCORING : currentState;

      case FEEDING_PREPARE_TO_SHOOT ->
          shooter.atGoal() && arm.atGoal() ? RobotState.FEEDING_SHOOTING : currentState;
      case PASS_PREPARE_TO_SHOOT ->
          shooter.atGoal() && arm.atGoal() ? RobotState.PASS_SHOOTING : currentState;
      case UNJAM -> currentState;
      case INTAKING -> queuer.hasNote() ? RobotState.IDLE_WITH_GP : currentState;
      case OUTTAKING -> queuer.hasNote() || intake.hasNote() ? currentState : RobotState.IDLE_NO_GP;
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case SPEAKER_PREPARE_TO_SCORE, SPEAKER_WAITING -> {
        arm.setState(ArmState.SPEAKER_SHOT);
        shooter.setState(ShooterState.SPEAKER_SHOT);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE_WITH_GP);
      }
      case SPEAKER_SCORING -> {
        arm.setState(ArmState.SPEAKER_SHOT);
        shooter.setState(ShooterState.SPEAKER_SHOT);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.SHOOTING);
      }
      case AMP_PREPARE_TO_SCORE, AMP_WAITING -> {
        arm.setState(ArmState.AMP);
        shooter.setState(ShooterState.AMP);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE_WITH_GP);
      }
      case AMP_SCORING -> {
        arm.setState(ArmState.AMP);
        shooter.setState(ShooterState.AMP);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.SHOOTING);
      }
      case FEEDING_PREPARE_TO_SHOOT, FEEDING_WAITING -> {
        arm.setState(ArmState.FEEDING);
        shooter.setState(ShooterState.FEEDING);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE_WITH_GP);
      }
      case FEEDING_SHOOTING -> {
        arm.setState(ArmState.FEEDING);
        shooter.setState(ShooterState.FEEDING);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.SHOOTING);
      }
      case PASS_PREPARE_TO_SHOOT, PASS_WAITING -> {
        arm.setState(ArmState.PASS);
        shooter.setState(ShooterState.PASS);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE_WITH_GP);
      }
      case PASS_SHOOTING -> {
        arm.setState(ArmState.PASS);
        shooter.setState(ShooterState.PASS);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.SHOOTING);
      }
      case UNJAM -> {
        arm.setState(ArmState.AMP);
        shooter.setState(ShooterState.PASS);
        intake.setState(IntakeState.OUTTAKING);
        queuer.setState(QueuerState.OUTTAKING);
      }
      case INTAKING -> {
        arm.setState(ArmState.IDLE);
        shooter.setState(ShooterState.IDLE_STOPPED);
        intake.setState(IntakeState.INTAKING);
        queuer.setState(QueuerState.INTAKING);
      }
      case OUTTAKING -> {
        arm.setState(ArmState.IDLE);
        shooter.setState(ShooterState.IDLE_STOPPED);
        intake.setState(IntakeState.OUTTAKING);
        queuer.setState(QueuerState.OUTTAKING);
      }
      case CLIMBING_1_LINEUP -> {
        arm.setState(ArmState.CLIMBING_1_LINEUP);
        shooter.setState(ShooterState.IDLE_STOPPED);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE_NO_GP);
      }
      case CLIMBING_2_HANGING -> {
        arm.setState(ArmState.CLIMBING_2_HANGING);
        shooter.setState(ShooterState.IDLE_STOPPED);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE_NO_GP);
      }
      case IDLE_NO_GP -> {
        arm.setState(ArmState.IDLE);
        shooter.setState(ShooterState.IDLE_STOPPED);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE_NO_GP);
      }
      case IDLE_WITH_GP -> {
        arm.setState(ArmState.IDLE);
        shooter.setState(ShooterState.IDLE_WARMUP);
        intake.setState(IntakeState.IDLE);
        queuer.setState(QueuerState.IDLE_WITH_GP);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    // Continuous state actions
    switch (getState()) {
      case SPEAKER_PREPARE_TO_SCORE, SPEAKER_SCORING, SPEAKER_WAITING -> {
        shooter.setDistanceToSpeaker(distanceToSpeaker);
        arm.setDistanceToSpeaker(distanceToSpeaker);
      }
      case FEEDING_PREPARE_TO_SHOOT, FEEDING_SHOOTING, FEEDING_WAITING -> {
        shooter.setDistanceToFeedSpot(distanceToFeedSpot);
        arm.setDistanceToFeedSpot(distanceToFeedSpot);
      }
      default -> {}
    }
  }
}
