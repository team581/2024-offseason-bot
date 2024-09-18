package frc.robot.imu;

import com.ctre.phoenix6.hardware.Pigeon2;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.VisionSubsystem;

public class ImuSubsystem extends StateMachine<ImuState> {
  private final Pigeon2 imu;
  private Rotation2d robotHeading = new Rotation2d();
  VisionSubsystem vision = new VisionSubsystem(null);
  private final LocalizationSubsystem localization = new LocalizationSubsystem(this, vision);


  public ImuSubsystem(Pigeon2 imu) {
    super(SubsystemPriority.IMU, ImuState.DEFAULT_STATE);
    this.imu = imu;
  }

  @Override
  protected void collectInputs() {
    robotHeading = Rotation2d.fromDegrees(imu.getYaw().getValue());
  }

  public Rotation2d getRobotHeading() {
    return robotHeading;
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Imu/RobotHeading", robotHeading.getDegrees());
  }
}
