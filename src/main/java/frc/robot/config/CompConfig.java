package frc.robot.config;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.config.RobotConfig.ArmConfig;
import frc.robot.config.RobotConfig.IntakeConfig;
import frc.robot.config.RobotConfig.QueuerConfig;
import frc.robot.config.RobotConfig.ShooterConfig;
import frc.robot.config.RobotConfig.SwerveConfig;
import frc.robot.config.RobotConfig.VisionConfig;
import frc.robot.vision.interpolation.InterpolatedVisionDataset;

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
              20,
              CANIVORE_NAME,
              0,
              new TalonFXConfiguration()
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(25)
                          .withStatorCurrentLimit(20))
                  .withSlot0(new Slot0Configs().withKV(0).withKP(1.0).withKI(0).withKD(0).withKG(0))
                  .withMotorOutput(
                      new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)),
              new Debouncer(3.0 * 0.02)),
          new ShooterConfig(
              18,
              17,
              CANIVORE_NAME,
              new TalonFXConfiguration()
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(40)
                          .withSupplyCurrentLimit(45))
                  .withSlot0(new Slot0Configs().withKV(0.15).withKP(3.5).withKI(0).withKD(0)),
              new TalonFXConfiguration()
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(40)
                          .withSupplyCurrentLimit(45))
                  .withSlot0(new Slot0Configs().withKV(0.15).withKP(3.5).withKI(0).withKD(0)),
              feedSpotDistanceToRpm -> {
                feedSpotDistanceToRpm.put(123.0, 321.0);
              },
              speakerDistanceToRpm -> {
                speakerDistanceToRpm.put(123.0, 321.0);
              }),
          new IntakeConfig(
              19,
              CANIVORE_NAME,
              2,
              new TalonFXConfiguration()
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(20)
                          .withSupplyCurrentLimit(25))
                  .withSlot0(
                      new Slot0Configs().withKV(0).withKP(1.0).withKI(0).withKD(0).withKG(0)),
              centeringMotor -> {
                centeringMotor.setSmartCurrentLimit(20);
                centeringMotor.burnFlash();
              }),
          new ArmConfig(
              CANIVORE_NAME,
              16,
              14,
              new TalonFXConfiguration()
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(40)
                          .withSupplyCurrentLimit(40))
                  .withSlot0(
                      new Slot0Configs()
                          .withKV(0)
                          .withKP(300.0)
                          .withKI(0)
                          .withKD(0)
                          .withKG(0.2)
                          .withGravityType(GravityTypeValue.Arm_Cosine))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio((84 * 84 * 24) / (8 * 20 * 9)))
                  .withMotorOutput(
                      new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicAcceleration(2.0)
                          .withMotionMagicCruiseVelocity(3.0)
                          .withMotionMagicJerk(100)),
              new TalonFXConfiguration()
                  .withClosedLoopRamps(CLOSED_LOOP_RAMP)
                  .withOpenLoopRamps(OPEN_LOOP_RAMP)
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(40)
                          .withSupplyCurrentLimit(40))
                  .withSlot0(
                      new Slot0Configs()
                          .withKV(0)
                          .withKP(300.0)
                          .withKI(0)
                          .withKD(0)
                          .withKG(0.2)
                          .withGravityType(GravityTypeValue.Arm_Cosine))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio((84 * 84 * 24) / (8 * 20 * 9)))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.CounterClockwise_Positive))
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicAcceleration(2.0)
                          .withMotionMagicCruiseVelocity(3.0)
                          .withMotionMagicJerk(100)),
              feedSpotDistanceToAngle -> {
                feedSpotDistanceToAngle.put(123.0, 321.0);
              },
              speakerDistanceToAngle -> {
                speakerDistanceToAngle.put(123.0, 321.0);
              },
              -77.0,
              87.0),
          new VisionConfig(4, 0.4, 0.4, InterpolatedVisionDataset.HOME));

  // NOT TUNED

  private CompConfig() {}
}
