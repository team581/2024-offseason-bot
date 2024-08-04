package frc.robot.config;

import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

public record RobotConfig(String robotName, String canivoreName, SwerveConfig swerve) {
  public record SwerveConfig(
      PhoenixPIDController snapController,
      boolean invertRotation,
      boolean invertX,
      boolean invertY) {}

  // TODO: Change this to false during events
  public static final boolean IS_DEVELOPMENT = true;
  public static final String SERIAL_NUMBER = System.getenv("serialnum");

  public static RobotConfig get() {
    return CompConfig.competitionBot;
  }
}
