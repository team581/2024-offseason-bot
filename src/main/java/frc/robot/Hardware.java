package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Power;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.config.RobotConfig;

public class Hardware {
    private final String canivoreName = null;

    public final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    public final TalonFX shooter = new TalonFX(RobotConfig.get().shooter().motorID(), canivoreName);
    
    public final TalonFX queuer = new TalonFX(RobotConfig.get().queuer().motorID(), canivoreName);


    public final TalonFX intake = new TalonFX(RobotConfig.get().intake().motorID(), canivoreName);

    public final TalonFX armLeft = new TalonFX(RobotConfig.get().arm().leftMotorID(), canivoreName);
        public final TalonFX armRight = new TalonFX(RobotConfig.get().arm().rightMotorID(), canivoreName);





}
