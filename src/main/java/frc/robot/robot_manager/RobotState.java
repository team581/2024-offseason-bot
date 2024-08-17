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

  UNJAM,

  PASS_WAITING,
  PASS_PREPARE_TO_SHOOT,
  PASS_SHOOTING;
}
