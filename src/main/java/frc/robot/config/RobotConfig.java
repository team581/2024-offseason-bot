package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.function.Consumer;

public record RobotConfig(
    String robotName,
    SwerveConfig swerve,
    QueuerConfig queuer,
    ShooterConfig shooter,
    IntakeConfig intake,
    ArmConfig arm) {
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

  public record ShooterConfig(
      int motorID,
      String canBusName,
      TalonFXConfiguration motorConfig,
      Consumer<InterpolatingDoubleTreeMap> feedSpotDistanceToRpm,
      Consumer<InterpolatingDoubleTreeMap> speakerDistanceToRpm) {}

  public record IntakeConfig(
      int motorID,
      String canBusName,
      int sensorID,
      TalonFXConfiguration motorConfig,
      Debouncer debouncer) {}

  public record ArmConfig(
      String canBusName,
      int leftMotorID,
      int rightMotorID,
      TalonFXConfiguration leftMotorConfig,
      TalonFXConfiguration rightMotorConfig,
      Consumer<InterpolatingDoubleTreeMap> feedSpotDistanceToAngle,
      Consumer<InterpolatingDoubleTreeMap> speakerDistanceToAngle,
      Double homingAngle) {}

  // TODO: Change this to false during events
  public static final boolean IS_DEVELOPMENT = true;
  public static final String SERIAL_NUMBER = System.getenv("serialnum");

  public static RobotConfig get() {
    return CompConfig.competitionBot;
  }
}
