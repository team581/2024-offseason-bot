package frc.robot.robot_manager;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class RobotCommands {
      private final RobotManager robot;
  private final Subsystem[] requirements;

  public RobotCommands(RobotManager robot) {
    this.robot = robot;
    var requirementsList =
        List.of(
            robot.arm,
            robot.intake,
            robot.queuer,
            robot.shooter);
    requirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command intakeCommand() {
    return Commands.runOnce(robot::intakeRequest, requirements).andThen(robot.waitForState(RobotState.IDLE_WITH_GP));
  }
  public Command outtakeCommand(){
    return Commands.runOnce(robot::outtakeRequest, requirements).andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }
  public Command speakerCommand(){
    return Commands.runOnce(robot::prepareSpeakerRequest, requirements).andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }
  public Command passCommand(){
    return Commands.runOnce(robot::preparePassRequest,requirements).andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command stowCommand(){
    return Commands.runOnce(robot::stowRequest,requirements).andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }
  public Command idleWithGpCommand(){
    return Commands.runOnce(robot::idleWithGpRequest,requirements).andThen(robot.waitForState(RobotState.IDLE_WITH_GP));
  }
  public Command stopIntakingCommand(){
        return Commands.runOnce(robot::stopIntakingRequest,requirements).andThen(robot.waitForState(RobotState.IDLE_NO_GP));

  }
  
  public Command waitSubwooferCommand(){
        return Commands.runOnce(robot::waitSubwooferRequest,requirements).andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }
    public Command waitAmpCommand(){
        return Commands.runOnce(robot::waitAmpRequest,requirements).andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }
  public Command waitSpeakerCommand(){
        return Commands.runOnce(robot::waitSpeakerRequest,requirements).andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }
  public Command confirmShotCommand(){
        return Commands.runOnce(robot::confirmShotRequest,requirements).andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }
  public Command ampCommand(){
        return Commands.runOnce(robot::prepareAmpRequest,requirements).andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }
  public Command stopShootingCommand(){
        return Commands.runOnce(robot::stopShootingRequest,requirements).andThen(robot.waitForState(RobotState.IDLE_WITH_GP));
  }
  public Command feedingCommand(){
    return Commands.runOnce(robot::prepareFeedRequest, requirements).andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }
  public Command subwooferCommand(){
        return Commands.runOnce(robot::prepareSubwooferRequest, requirements).andThen(robot.waitForState(RobotState.IDLE_NO_GP));

  }
}
