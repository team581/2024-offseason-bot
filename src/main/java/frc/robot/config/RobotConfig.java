package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.filter.Debouncer;

public record RobotConfig(String robotName, SwerveConfig swerve, QueuerConfig queuer) {
  public record SwerveConfig(
      PhoenixPIDController snapController,
      boolean invertRotation,
      boolean invertX,
      boolean invertY) {}

  public record QueuerConfig(
      int motorID,
      String canBusName,
      int sensorID,
      TalonFXConfiguration motorConfig,
      Debouncer debouncer) {}

  // TODO: Change this to false during events
  public static final boolean IS_DEVELOPMENT = true;
  public static final String SERIAL_NUMBER = System.getenv("serialnum");

  public static RobotConfig get() {
    return CompConfig.competitionBot;
  }
}
