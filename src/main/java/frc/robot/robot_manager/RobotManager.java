package frc.robot.robot_manager;

import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class RobotManager extends StateMachine<RobotState> {
  public RobotManager() {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE_NO_GP);
  }

  @Override
  protected void collectInputs() {}

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
  }
}
