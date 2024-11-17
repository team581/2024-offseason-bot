package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.trailblazer.Trailblazer;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;

public abstract class BaseAuto {
  protected final RobotManager robotManager;
  protected final AutoCommands autoCommands;
  protected final RobotCommands actions;
  protected final Trailblazer trailblazer;

  protected BaseAuto(RobotManager robotManager, Trailblazer trailblazer) {
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;
    this.actions = new RobotCommands(robotManager);
    this.autoCommands = new AutoCommands(actions, robotManager);
  }

  public abstract Command getAutoCommand();
}
