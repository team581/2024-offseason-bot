package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.config.RobotConfig;

public class Hardware {
  public final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  public final TalonFX shooter =
      new TalonFX(RobotConfig.get().shooter().motorID(), RobotConfig.get().shooter().canBusName());

  public final TalonFX queuer =
      new TalonFX(RobotConfig.get().queuer().motorID(), RobotConfig.get().queuer().canBusName());
  public final DigitalInput queuerSensor = new DigitalInput(RobotConfig.get().queuer().sensorID());

  public final TalonFX intakeMain =
      new TalonFX(
          RobotConfig.get().intake().mainMotorID(),
          RobotConfig.get().intake().mainMotorCanBusName());

  public final TalonFX armLeft =
      new TalonFX(RobotConfig.get().arm().leftMotorID(), RobotConfig.get().arm().canBusName());
  public final TalonFX armRight =
      new TalonFX(RobotConfig.get().arm().rightMotorID(), RobotConfig.get().arm().canBusName());
  public final Pigeon2 imu =
      new Pigeon2(RobotConfig.get().imu().pigeonID(), RobotConfig.get().imu().canBusName());
}
