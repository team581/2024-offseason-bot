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

  protected RobotState getNexState(RobotState currentState) {
    // state transition
    switch (currentState) {
      case SPEAKER_SCORING -> {
        if (!queuer.hasNote()) {
          setStateFromRequest(RobotState.IDLE_NO_GP);
        }
      }
      case AMP_SCORING -> {
        if (!queuer.hasNote()) {
          setStateFromRequest(RobotState.IDLE_NO_GP);
        }
      }
      case FEEDING_SHOOTING -> {
        if (!queuer.hasNote()) {
          setStateFromRequest(RobotState.IDLE_NO_GP);
        }
      }
      case PASS_SHOOTING -> {
        if (!queuer.hasNote()) {
          setStateFromRequest(RobotState.IDLE_NO_GP);
        }
      }
      case SPEAKER_PREPARE_TO_SCORE -> {
        if (shooter.atGoal() && arm.atGoal()) {
          setStateFromRequest(RobotState.SPEAKER_SCORING);
        }
      }

      case AMP_PREPARE_TO_SCORE -> {
        if (shooter.atGoal() && arm.atGoal()) {
          setStateFromRequest(RobotState.AMP_SCORING);
        }
      }
      case FEEDING_PREPARE_TO_SHOOT -> {
        if (shooter.atGoal() && arm.atGoal()) {
          setStateFromRequest(RobotState.FEEDING_SHOOTING);
        }
      }
      case PASS_PREPARE_TO_SHOOT -> {
        if (shooter.atGoal() && arm.atGoal()) {
          setStateFromRequest(RobotState.PASS_SHOOTING);
        }
      }
      case UNJAM -> {}
      case INTAKING -> {
        if (intake.hasNote() || queuer.hasNote()) {
          setStateFromRequest(RobotState.IDLE_WITH_GP);
        }
      }
      case OUTTAKING -> {
        if (!intake.hasNote() && !queuer.hasNote()) {
          setStateFromRequest(RobotState.IDLE_NO_GP);
        }
      }
      case CLIMBING_1_LINEUP -> {
        if (arm.atGoal()) {
          setStateFromRequest(RobotState.CLIMBING_2_HANGING);
        }
      }
      case CLIMBING_2_HANGING -> {}
      case IDLE_NO_GP -> {
        if (queuer.hasNote() || intake.hasNote()) {
          setStateFromRequest(RobotState.IDLE_WITH_GP);
        }
      }
      case IDLE_WITH_GP -> {
        if (!queuer.hasNote() && !intake.hasNote()) {
          setStateFromRequest(RobotState.IDLE_NO_GP);
        }
      }
    }
    return currentState;
  }

  @Override
  protected void afterTransition(RobotState newState) {
    // on state change
    switch (newState) {
      case SPEAKER_PREPARE_TO_SCORE, SPEAKER_SCORING -> {
        arm.setState(ArmState.SPEAKER_SHOT);
        shooter.setState(ShooterState.SPEAKER_SHOT);
        intake.setState(IntakeState.IDLE);
        queuer.seState(QueuerState.IDLE_WITH_GP);
      }
      case AMP_PREPARE_TO_SCORE, AMP_SCORING -> {
        arm.setState(ArmState.AMP);
        shooter.setState(ShooterState.AMP);
        intake.setState(IntakeState.IDLE);
        queuer.seState(QueuerState.IDLE_WITH_GP);
      }
      case FEEDING_PREPARE_TO_SHOOT, FEEDING_SHOOTING -> {
        arm.setState(ArmState.FEEDING);
        shooter.setState(ShooterState.FEEDING);
        intake.setState(IntakeState.IDLE);
        queuer.seState(QueuerState.IDLE_WITH_GP);
      }
      case PASS_PREPARE_TO_SHOOT, PASS_SHOOTING -> {
        arm.setState(ArmState.PASS);
        shooter.setState(ShooterState.PASS);
        intake.setState(IntakeState.IDLE);
        queuer.seState(QueuerState.IDLE_WITH_GP);
      }
      case UNJAM -> {
        arm.setState(ArmState.AMP);
        shooter.setState(ShooterState.DROP);
        intake.setState(IntakeState.OUTTAKING);
        queuer.seState(QueuerState.OUTTAKING);
      }
      case INTAKING -> {
        arm.setState(ArmState.IDLE);
        shooter.setState(ShooterState.IDLE_STOPPED);
        intake.setState(IntakeState.INTAKING);
        queuer.seState(QueuerState.INTAKING);
      }
      case OUTTAKING -> {
        arm.setState(ArmState.IDLE);
        shooter.setState(ShooterState.IDLE_STOPPED);
        intake.setState(IntakeState.OUTTAKING);
        queuer.seState(QueuerState.OUTTAKING);
      }
      case CLIMBING_1_LINEUP -> {
        arm.setState(ArmState.CLIMBING_1_LINEUP);
        shooter.setState(ShooterState.IDLE_STOPPED);
        intake.setState(IntakeState.IDLE);
        queuer.seState(QueuerState.IDLE_NO_GP);
      }
      case CLIMBING_2_HANGING -> {
        arm.setState(ArmState.CLIMBING_1_LINEUP);
        shooter.setState(ShooterState.IDLE_STOPPED);
        intake.setState(IntakeState.IDLE);
        queuer.seState(QueuerState.IDLE_NO_GP);
      }
      case IDLE_NO_GP -> {
        arm.setState(ArmState.IDLE);
        shooter.setState(ShooterState.IDLE_STOPPED);
        intake.setState(IntakeState.IDLE);
        queuer.seState(QueuerState.IDLE_NO_GP);
      }
      case IDLE_WITH_GP -> {
        arm.setState(ArmState.IDLE);
        shooter.setState(ShooterState.IDLE_STOPPED);
        intake.setState(IntakeState.IDLE);
        queuer.seState(QueuerState.IDLE_WITH_GP);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    // continous sate action
    switch (getState()) {
      case SPEAKER_PREPARE_TO_SCORE, SPEAKER_SCORING -> {
        shooter.setDistanceToSpeaker(distanceToSpeaker);
        arm.setDistanceToSpeaker(distanceToSpeaker);
      }
      case FEEDING_PREPARE_TO_SHOOT, FEEDING_SHOOTING -> {
        shooter.setDistanceToFeedSpot(distanceToFeedSpot);
        arm.setDistanceToFeedSpot(distanceToFeedSpot);
      }

      case AMP_PREPARE_TO_SCORE,
          PASS_PREPARE_TO_SHOOT,
          AMP_SCORING,
          PASS_SHOOTING,
          UNJAM,
          INTAKING,
          OUTTAKING,
          IDLE_NO_GP,
          IDLE_WITH_GP,
          CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING -> {}
    }
  }
}
