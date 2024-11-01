package frc.robot.robot_manager;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldUtil;
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
import frc.robot.swerve.SnapUtil;
import frc.robot.swerve.SwerveState;
import frc.robot.swerve.SwerveSubsystem;
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
  public final SwerveSubsystem swerve;

  private final double distanceToFeedSpot = 0.0;
  private final double distanceToSpeaker = 0.0;

  private boolean confirmShotActive = false;
  private double fieldRelativeAngleToSpeaker = 0;
  private double fieldRelativeAngleToFeedSpot = 0;

  public RobotManager(
      ArmSubsystem arm,
      ShooterSubsystem shooter,
      LocalizationSubsystem localization,
      VisionSubsystem vision,
      ImuSubsystem imu,
      IntakeSubsystem intake,
      QueuerSubsystem queuer,
      SwerveSubsystem swerve) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE_NO_GP);
    this.arm = arm;
    this.shooter = shooter;
    this.localization = localization;
    this.vision = vision;
    this.imu = imu;
    this.intake = intake;
    this.queuer = queuer;
    this.swerve = swerve;

    // Idle
    createHandler(RobotState.IDLE_NO_GP)
        .onEnter(
            () -> {
              arm.setState(ArmState.IDLE);
              shooter.setState(ShooterState.IDLE_STOPPED);
              intake.setState(IntakeState.IDLE);
              queuer.setState(QueuerState.IDLE);
              swerve.setSnapsEnabled(false);
              swerve.setSnapToAngle(0);
            });
    createHandler(RobotState.IDLE_WITH_GP)
        .onEnter(
            () -> {
              arm.setState(ArmState.IDLE);
              shooter.setState(ShooterState.IDLE_WARMUP);
              intake.setState(IntakeState.IDLE);
              queuer.setState(QueuerState.IDLE);
              swerve.setSnapsEnabled(false);
              swerve.setSnapToAngle(0);
            });

    // Subwoofer
    createHandler(RobotState.SUBWOOFER_WAITING)
        .onEnter(
            () -> {
              arm.setState(ArmState.SUBWOOFER_SHOT);
              shooter.setState(ShooterState.SUBWOOFER_SHOT);
              intake.setState(IntakeState.IDLE);
              queuer.setState(QueuerState.IDLE);
              swerve.setSnapsEnabled(true);
              swerve.setSnapToAngle(SnapUtil.getSubwooferAngle());
            });
    createHandler(RobotState.SUBWOOFER_PREPARE_TO_SCORE)
        .onEnter(RobotState.SUBWOOFER_WAITING)
        .withTransitions(
            () -> shooter.atGoal() && arm.atGoal() ? RobotState.SUBWOOFER_SCORING : null);
    createHandler(RobotState.SUBWOOFER_SCORING)
        .onEnter(
            () -> {
              arm.setState(ArmState.SUBWOOFER_SHOT);
              shooter.setState(ShooterState.SUBWOOFER_SHOT);
              intake.setState(IntakeState.IDLE);
              queuer.setState(QueuerState.SHOOTING);
              swerve.setSnapsEnabled(true);
              swerve.setSnapToAngle(SnapUtil.getSubwooferAngle());
            })
        .withTransitions(() -> queuer.hasNote() ? null : RobotState.IDLE_NO_GP);

    // Podium
    createHandler(RobotState.PODIUM_WAITING)
        .onEnter(
            () -> {
              arm.setState(ArmState.PODIUM_SHOT);
              shooter.setState(ShooterState.PODIUM_SHOT);
              intake.setState(IntakeState.IDLE);
              queuer.setState(QueuerState.IDLE);
              swerve.setSnapsEnabled(true);
              swerve.setSnapToAngle(SnapUtil.getPodiumAngle());
            });
    createHandler(RobotState.PODIUM_PREPARE_TO_SCORE)
        .onEnter(RobotState.PODIUM_WAITING)
        .withTransitions(() -> shooter.atGoal() && arm.atGoal() ? RobotState.PODIUM_SCORING : null);
    createHandler(RobotState.PODIUM_SCORING)
        .onEnter(
            () -> {
              arm.setState(ArmState.PODIUM_SHOT);
              shooter.setState(ShooterState.PODIUM_SHOT);
              intake.setState(IntakeState.IDLE);
              queuer.setState(QueuerState.SHOOTING);
              swerve.setSnapsEnabled(true);
              swerve.setSnapToAngle(SnapUtil.getPodiumAngle());
            })
        .withTransitions(() -> queuer.hasNote() ? null : RobotState.IDLE_NO_GP);

    // Speaker
    createHandler(RobotState.SPEAKER_PREPARE_TO_SCORE)
        .onEnter(
            () -> {
              arm.setState(ArmState.SPEAKER_SHOT);
              shooter.setState(ShooterState.SPEAKER_SHOT);
              intake.setState(IntakeState.IDLE);
              queuer.setState(QueuerState.IDLE);
              swerve.setSnapsEnabled(true);
              swerve.setSnapToAngle(fieldRelativeAngleToSpeaker);
            })
        .withPeriodic(
            () -> {
              shooter.setDistanceToSpeaker(distanceToSpeaker);
              arm.setDistanceToSpeaker(distanceToSpeaker);
              swerve.setSnapToAngle(fieldRelativeAngleToSpeaker);
            })
        .withTransitions(
            () ->
                (shooter.atGoal() && arm.atGoal())
                        && (DriverStation.isAutonomous() || confirmShotActive == true)
                    ? RobotState.SPEAKER_SCORING
                    : null);
    createHandler(RobotState.SPEAKER_WAITING)
        .onEnter(RobotState.SPEAKER_PREPARE_TO_SCORE)
        .withPeriodic(RobotState.SPEAKER_PREPARE_TO_SCORE);
    createHandler(RobotState.SPEAKER_SCORING)
        .onEnter(
            () -> {
              arm.setState(ArmState.SPEAKER_SHOT);
              shooter.setState(ShooterState.SPEAKER_SHOT);
              intake.setState(IntakeState.IDLE);
              queuer.setState(QueuerState.SHOOTING);
              swerve.setSnapsEnabled(true);
              swerve.setSnapToAngle(fieldRelativeAngleToSpeaker);
            })
        .withPeriodic(RobotState.SPEAKER_PREPARE_TO_SCORE)
        .withTransitions(() -> queuer.hasNote() ? null : RobotState.IDLE_NO_GP);

    // Amp
    createHandler(RobotState.AMP_PREPARE_TO_SCORE)
        .onEnter(
            () -> {
              arm.setState(ArmState.AMP);
              shooter.setState(ShooterState.IDLE_WARMUP);
              intake.setState(IntakeState.IDLE);
              queuer.setState(QueuerState.IDLE);
              swerve.setSnapsEnabled(true);
              swerve.setSnapToAngle(SnapUtil.getAmpAngle());
            })
        .withTransitions(() -> shooter.atGoal() && arm.atGoal() ? RobotState.AMP_SCORING : null);
    createHandler(RobotState.AMP_WAITING).onEnter(RobotState.AMP_PREPARE_TO_SCORE);
    createHandler(RobotState.AMP_SCORING)
        .onEnter(
            () -> {
              arm.setState(ArmState.AMP);
              shooter.setState(ShooterState.IDLE_WARMUP);
              intake.setState(IntakeState.IDLE);
              queuer.setState(QueuerState.AMPING);
              swerve.setSnapsEnabled(true);
              swerve.setSnapToAngle(SnapUtil.getAmpAngle());
            })
        .withTransitions(() -> queuer.hasNote() ? null : RobotState.IDLE_NO_GP);

    // Feeding
    createHandler(RobotState.FEEDING_PREPARE_TO_SHOOT)
        .onEnter(
            () -> {
              arm.setState(ArmState.FEEDING);
              shooter.setState(ShooterState.FEEDING);
              intake.setState(IntakeState.IDLE);
              queuer.setState(QueuerState.IDLE);
              swerve.setSnapsEnabled(true);
              swerve.setSnapToAngle(fieldRelativeAngleToFeedSpot);
            })
        .withPeriodic(
            () -> {
              shooter.setDistanceToFeedSpot(distanceToFeedSpot);
              arm.setDistanceToFeedSpot(distanceToFeedSpot);
            });
    createHandler(RobotState.FEEDING_WAITING)
        .onEnter(RobotState.FEEDING_PREPARE_TO_SHOOT)
        .withPeriodic(RobotState.FEEDING_PREPARE_TO_SHOOT)
        .withTransitions(
            () -> shooter.atGoal() && arm.atGoal() ? RobotState.FEEDING_SHOOTING : null);
    createHandler(RobotState.FEEDING_SHOOTING)
        .onEnter(
            () -> {
              arm.setState(ArmState.FEEDING);
              shooter.setState(ShooterState.FEEDING);
              intake.setState(IntakeState.IDLE);
              queuer.setState(QueuerState.SHOOTING);
              swerve.setSnapsEnabled(true);
              swerve.setSnapToAngle(fieldRelativeAngleToFeedSpot);
            })
        .withPeriodic(RobotState.FEEDING_PREPARE_TO_SHOOT)
        .withTransitions(() -> queuer.hasNote() ? null : RobotState.IDLE_NO_GP);

    // Passing
    createHandler(RobotState.PASS_PREPARE_TO_SHOOT)
        .onEnter(
            () -> {
              arm.setState(ArmState.IDLE);
              shooter.setState(ShooterState.PASS);
              intake.setState(IntakeState.IDLE);
              queuer.setState(QueuerState.IDLE);
              swerve.setSnapsEnabled(false);
              swerve.setSnapToAngle(0);
            })
        .withTransitions(() -> shooter.atGoal() && arm.atGoal() ? RobotState.PASS_SHOOTING : null);
    createHandler(RobotState.PASS_SHOOTING)
        .onEnter(
            () -> {
              arm.setState(ArmState.IDLE);
              shooter.setState(ShooterState.PASS);
              intake.setState(IntakeState.IDLE);
              queuer.setState(QueuerState.SHOOTING);
              swerve.setSnapsEnabled(false);
              swerve.setSnapToAngle(0);
            })
        .withTransitions(() -> queuer.hasNote() ? null : RobotState.IDLE_NO_GP);

    // Unjam
    createHandler(RobotState.UNJAM)
        .onEnter(
            () -> {
              arm.setState(ArmState.AMP);
              shooter.setState(ShooterState.PASS);
              intake.setState(IntakeState.OUTTAKING);
              queuer.setState(QueuerState.OUTTAKING);
              swerve.setSnapsEnabled(false);
              swerve.setSnapToAngle(0);
            });

    // Intaking
    createHandler(RobotState.INTAKING)
        .onEnter(
            () -> {
              arm.setState(ArmState.IDLE);
              shooter.setState(ShooterState.IDLE_STOPPED);
              intake.setState(IntakeState.INTAKING);
              queuer.setState(QueuerState.INTAKING);
              swerve.setSnapsEnabled(false);
              swerve.setSnapToAngle(0);
            })
        .withPeriodic(
            () -> {
              if (arm.atGoal()) {
                intake.setState(IntakeState.INTAKING);
              } else {
                intake.setState(IntakeState.IDLE);
              }
            })
        .withTransitions(() -> queuer.hasNote() ? RobotState.INTAKING_BACK : null);
    createHandler(RobotState.INTAKE_ASSIST)
        .onEnter(
            () -> {
              arm.setState(ArmState.IDLE);
              shooter.setState(ShooterState.IDLE_STOPPED);
              intake.setState(IntakeState.INTAKING);
              queuer.setState(QueuerState.INTAKING);
              // We don't use the setSnaps here, since intake assist is a separate state
              if (DriverStation.isTeleop()) {
                swerve.setState(SwerveState.INTAKE_ASSIST_TELEOP);
              } else {
                swerve.setState(SwerveState.INTAKE_ASSIST_AUTO);
              }
            })
        .withPeriodic(RobotState.INTAKING)
        .withTransitions(RobotState.INTAKING);
    createHandler(RobotState.INTAKING_BACK)
        .onEnter(
            () -> {
              arm.setState(ArmState.IDLE);
              shooter.setState(ShooterState.IDLE_STOPPED);
              intake.setState(IntakeState.INTAKING_BACK);
              queuer.setState(QueuerState.INTAKING_BACK);
              swerve.setSnapsEnabled(false);
              swerve.setSnapToAngle(0);
            })
        .withPeriodic(
            () -> {
              if (arm.atGoal()) {
                intake.setState(IntakeState.INTAKING_BACK);
              } else {
                intake.setState(IntakeState.IDLE);
              }
            })
        .withTransitions(() -> queuer.hasNote() ? null : RobotState.INTAKING_FORWARD_PUSH);
    createHandler(RobotState.INTAKING_FORWARD_PUSH)
        .onEnter(
            () -> {
              arm.setState(ArmState.IDLE);
              shooter.setState(ShooterState.IDLE_STOPPED);
              intake.setState(IntakeState.INTAKING_FORWARD_PUSH);
              queuer.setState(QueuerState.INTAKING_FORWARD_PUSH);
              swerve.setSnapsEnabled(false);
              swerve.setSnapToAngle(0);
            })
        .withPeriodic(
            () -> {
              if (arm.atGoal()) {
                intake.setState(IntakeState.INTAKING_FORWARD_PUSH);
              } else {
                intake.setState(IntakeState.IDLE);
              }
            })
        .withTransitions(() -> queuer.hasNote() ? RobotState.IDLE_WITH_GP : null);

    // Outtake
    createHandler(RobotState.OUTTAKING)
        .onEnter(
            () -> {
              arm.setState(ArmState.IDLE);
              shooter.setState(ShooterState.IDLE_STOPPED);
              intake.setState(IntakeState.OUTTAKING);
              queuer.setState(QueuerState.OUTTAKING);
              swerve.setSnapsEnabled(false);
              swerve.setSnapToAngle(0);
            });

    // Climbing
    createHandler(RobotState.CLIMBING_1_LINEUP)
        .onEnter(
            () -> {
              arm.setState(ArmState.CLIMBING_1_LINEUP);
              shooter.setState(ShooterState.IDLE_STOPPED);
              intake.setState(IntakeState.IDLE);
              queuer.setState(QueuerState.IDLE);
              swerve.setSnapsEnabled(true);
              swerve.setSnapToAngle(SnapUtil.getClimbingAngle(imu));
            });
    createHandler(RobotState.CLIMBING_2_HANGING)
        .onEnter(
            () -> {
              arm.setState(ArmState.CLIMBING_2_HANGING);
              shooter.setState(ShooterState.IDLE_STOPPED);
              intake.setState(IntakeState.IDLE);
              queuer.setState(QueuerState.IDLE);
              swerve.setSnapsEnabled(false);
              swerve.setSnapToAngle(0);
            });
  }

  @Override
  protected void collectInputs() {
    fieldRelativeAngleToSpeaker =
        localization.getFieldRelativeAngleToPose(FieldUtil.getSpeakerPose());
    fieldRelativeAngleToFeedSpot =
        localization.getFieldRelativeAngleToPose(FieldUtil.getFeedSpotPose());
  }

  public void setConfirmShotActive(boolean newValue) {
    confirmShotActive = newValue;
  }

  public void confirmShotRequest() {

    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}

      case AMP_WAITING -> setStateFromRequest(RobotState.AMP_PREPARE_TO_SCORE);
      case SPEAKER_WAITING -> setStateFromRequest(RobotState.SPEAKER_PREPARE_TO_SCORE);
      case FEEDING_WAITING -> setStateFromRequest(RobotState.FEEDING_PREPARE_TO_SHOOT);
      case SUBWOOFER_WAITING -> setStateFromRequest(RobotState.SUBWOOFER_PREPARE_TO_SCORE);
      case PODIUM_WAITING -> setStateFromRequest(RobotState.PODIUM_PREPARE_TO_SCORE);
      default -> setStateFromRequest(RobotState.SPEAKER_PREPARE_TO_SCORE);
    }
  }

  public void waitAmpRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          AMP_SCORING,
          INTAKING_BACK,
          INTAKING_FORWARD_PUSH,
          INTAKING -> {}
      default -> setStateFromRequest(RobotState.AMP_WAITING);
    }
  }

  public void waitSubwooferRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          SPEAKER_SCORING,
          INTAKING_BACK,
          INTAKING_FORWARD_PUSH,
          INTAKING -> {}
      default -> setStateFromRequest(RobotState.SUBWOOFER_WAITING);
    }
  }

  public void waitPodiumRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          SPEAKER_SCORING,
          INTAKING_BACK,
          INTAKING_FORWARD_PUSH,
          INTAKING -> {}
      default -> setStateFromRequest(RobotState.PODIUM_WAITING);
    }
  }

  public void waitSpeakerRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          INTAKING_BACK,
          INTAKING_FORWARD_PUSH,
          INTAKING -> {}
      default -> setStateFromRequest(RobotState.SPEAKER_WAITING);
    }
  }

  public void unjamRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.UNJAM);
    }
  }

  public void intakeRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.INTAKING);
    }
  }

  public void intakeAssistRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.INTAKE_ASSIST);
    }
  }

  public void outtakeRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.OUTTAKING);
    }
  }

  public void idleWithGpRequest() {
    setStateFromRequest(RobotState.IDLE_WITH_GP);
  }

  public void stowRequest() {
    switch (getState()) {
        // TODO: Intaking and unjam should not be IDLE_WITH_GP
      case INTAKING,
              AMP_PREPARE_TO_SCORE,
              SPEAKER_PREPARE_TO_SCORE,
              FEEDING_PREPARE_TO_SHOOT,
              PASS_PREPARE_TO_SHOOT,
              AMP_WAITING,
              SPEAKER_WAITING,
              FEEDING_WAITING,
              AMP_SCORING,
              SPEAKER_SCORING,
              FEEDING_SHOOTING,
              PASS_SHOOTING,
              IDLE_WITH_GP,
              UNJAM,
              INTAKING_BACK,
              INTAKING_FORWARD_PUSH ->
          setStateFromRequest(RobotState.IDLE_WITH_GP);
      default -> setStateFromRequest(RobotState.IDLE_NO_GP);
    }
  }

  public void preparePassRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          INTAKING_BACK,
          INTAKING_FORWARD_PUSH,
          INTAKING -> {}
      default -> setStateFromRequest(RobotState.PASS_PREPARE_TO_SHOOT);
    }
  }

  public void nextClimbStateRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP -> setStateFromRequest(RobotState.CLIMBING_2_HANGING);
      case CLIMBING_2_HANGING -> {}
      default -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
    }
  }

  public void previousClimbStateRequest() {
    switch (getState()) {
      case CLIMBING_2_HANGING -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
      case CLIMBING_1_LINEUP -> {
        setStateFromRequest(RobotState.IDLE_WITH_GP);
      }
      default -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
    }
  }

  public void prepareSpeakerRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          INTAKING_BACK,
          INTAKING_FORWARD_PUSH,
          INTAKING -> {}
      default -> setStateFromRequest(RobotState.SPEAKER_PREPARE_TO_SCORE);
    }
  }

  public void prepareAmpRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          INTAKING_BACK,
          INTAKING_FORWARD_PUSH,
          INTAKING -> {}
      default -> setStateFromRequest(RobotState.AMP_PREPARE_TO_SCORE);
    }
  }

  public void prepareFeedRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          INTAKING_BACK,
          INTAKING_FORWARD_PUSH,
          INTAKING -> {}
      default -> setStateFromRequest(RobotState.FEEDING_PREPARE_TO_SHOOT);
    }
  }

  public void waitFeedRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          INTAKING_BACK,
          INTAKING_FORWARD_PUSH,
          INTAKING -> {}
      default -> setStateFromRequest(RobotState.FEEDING_WAITING);
    }
  }

  public void stopShootingRequest() {
    // If we are actively taking a shot, ignore the request to avoid messing up shooting
    switch (getState()) {
      case SPEAKER_SCORING,
          SUBWOOFER_SCORING,
          AMP_SCORING,
          AMP_WAITING,
          FEEDING_SHOOTING,
          PASS_SHOOTING -> {}

      default -> setStateFromRequest(RobotState.IDLE_WITH_GP);
    }
  }

  public void prepareSubwooferRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          INTAKING_BACK,
          INTAKING_FORWARD_PUSH,
          INTAKING -> {}
      default -> setStateFromRequest(RobotState.SUBWOOFER_PREPARE_TO_SCORE);
    }
  }

  public void preparePodiumRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          INTAKING_BACK,
          INTAKING_FORWARD_PUSH,
          INTAKING -> {}
      default -> setStateFromRequest(RobotState.PODIUM_PREPARE_TO_SCORE);
    }
  }
}
