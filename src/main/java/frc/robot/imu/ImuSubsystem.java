package frc.robot.imu;

import com.ctre.phoenix6.hardware.Pigeon2;
import dev.doglog.DogLog;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ImuSubsystem extends StateMachine<ImuState> {
  private final Pigeon2 imu;
  private double robotHeading = 0;

  public ImuSubsystem(Pigeon2 imu) {
    super(SubsystemPriority.IMU, ImuState.DEFAULT_STATE);
    this.imu = imu;
  }

  @Override
  protected void collectInputs() {
    robotHeading = imu.getYaw().getValue();
  }

  public double getRobotHeading() {
    return robotHeading;
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Imu/RobotHeading", robotHeading);
  }
}
