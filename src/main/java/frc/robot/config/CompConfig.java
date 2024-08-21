package frc.robot.config;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.config.RobotConfig.IntakeConfig;
import frc.robot.config.RobotConfig.QueuerConfig;
import frc.robot.config.RobotConfig.ShooterConfig;
import frc.robot.config.RobotConfig.SwerveConfig;

class CompConfig {
  private static final String CANIVORE_NAME = "581CANivore";
  private static final String RIO_CAN_NAME = "rio";

  private static final ClosedLoopRampsConfigs CLOSED_LOOP_RAMP =
      new ClosedLoopRampsConfigs()
          .withDutyCycleClosedLoopRampPeriod(0.04)
          .withTorqueClosedLoopRampPeriod(0.04)
          .withVoltageClosedLoopRampPeriod(0.04);
  private static final OpenLoopRampsConfigs OPEN_LOOP_RAMP =
      new OpenLoopRampsConfigs()
          .withDutyCycleOpenLoopRampPeriod(0.04)
          .withTorqueOpenLoopRampPeriod(0.04)
          .withVoltageOpenLoopRampPeriod(0.04);

  public static final RobotConfig competitionBot =
      new RobotConfig(
          "competition",
          new SwerveConfig(new PhoenixPIDController(20, 0, 2), true, true, true),
          new QueuerConfig(
              999,
              CANIVORE_NAME,
              999,
              new TalonFXConfiguration()
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(25)
                          .withStatorCurrentLimit(20)),
              new Debouncer(3.0 * 0.02)),
          new ShooterConfig(
              999,
              CANIVORE_NAME,
              new TalonFXConfiguration()
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(40)
                          .withSupplyCurrentLimit(45))
                  .withSlot0(new Slot0Configs().withKV(0).withKP(0).withKI(0).withKD(0)),
              feedSpotDistanceToRpm -> {
                feedSpotDistanceToRpm.put(123.0, 321.0);
              },
              speakerDistanceToRpm -> {
                speakerDistanceToRpm.put(123.0, 321.0);
              }),
          new IntakeConfig(
              999,
              CANIVORE_NAME,
              999,
              new TalonFXConfiguration()
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(20)
                          .withSupplyCurrentLimit(25)),
              new Debouncer(3.0 * 0.02)));

  private CompConfig() {}
}
