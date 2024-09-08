package frc.robot.util.scheduling;

public enum SubsystemPriority {
  // 20-29 is for sensor subsystems

  // IMU runs before vision so that it has fresh data to pass to MegaTag2
  IMU(22),
  // Vision runs before localization so that it has fresh vision data for pose estimator
  VISION(21),
  // Localization runs before arm and shooter so that they have fresh speaker distance values
  LOCALIZATION(20),
  FMS(20),

  // 10-19 is for actuator subsystems

  ARM(10),
  SHOOTER(10),
  QUEUER(10),
  INTAKE(10),

  // 0-9 is for manager subsystems

  // Robot manager runs last so that all sensor data is fresh before processing state transitions
  ROBOT_MANAGER(0),
  AUTOS(0);

  final int value;

  private SubsystemPriority(int priority) {
    this.value = priority;
  }
}
