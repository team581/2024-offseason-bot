package frc.robot.robot_manager;

public enum RobotState {
  IDLE_WITH_GP,
  IDLE_NO_GP,

  AMP_WAITING,
  AMP_PREPARE_TO_SCORE,
  AMP_SCORING,

  SPEAKER_WAITING,
  SPEAKER_PREPARE_TO_SCORE,
  SPEAKER_SCORING,

  FEEDING_WAITING,
  FEEDING_PREPARE_TO_SHOOT,
  FEEDING_SHOOTING,

  INTAKING,
  OUTTAKING,
  INTAKE_ASSIST,

  UNJAM,

  PASS_PREPARE_TO_SHOOT,
  PASS_SHOOTING,

  CLIMBING_1_LINEUP,
  CLIMBING_2_HANGING,

  SUBWOOFER_WAITING,
  SUBWOOFER_PREPARE_TO_SCORE,
  SUBWOOFER_SCORING;
}
