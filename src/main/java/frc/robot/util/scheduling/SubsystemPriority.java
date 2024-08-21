package frc.robot.util.scheduling;

public enum SubsystemPriority {
  // Vision must run before **everything** to ensure that the cached data is fresh
  VISION(50),

  NOTE_TRACKING(41),
  AUTOS(40),

  ROBOT_MANAGER(30),

  SNAPS(20),

  SWERVE(10),
  IMU(10),
  SHOOTER(10),
  LOCALIZATION(10),
  INTAKE(10),
  QUEUER(10),
  ARM(10),

  FMS(0),
  RUMBLE_CONTROLLER(0);

  final int value;

  private SubsystemPriority(int priority) {
    this.value = priority;
  }
}
