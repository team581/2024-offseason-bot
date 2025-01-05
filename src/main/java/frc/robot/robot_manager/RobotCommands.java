package frc.robot.robot_manager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.autos.trailblazer.AutoPoint;
import frc.robot.autos.trailblazer.AutoSegment;
import frc.robot.autos.trailblazer.Trailblazer;
import java.util.List;

public class RobotCommands {
  private final RobotManager robot;
  private final Subsystem[] requirements;
  private final Trailblazer trailblazer;

  public RobotCommands(RobotManager robot, Trailblazer trailblazer) {
    this.robot = robot;
    this.trailblazer = trailblazer;
    var requirementsList = List.of(robot.arm, robot.intake, robot.queuer, robot.shooter);
    requirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command intakeCommand() {
    return Commands.runOnce(robot::intakeRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_WITH_GP));
  }

  public Command intakeAssistCommand() {
    return Commands.runOnce(robot::intakeAssistRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_WITH_GP))
        .withName("IntakeAssistCommand");
  }

  public Command outtakeCommand() {
    return Commands.runOnce(robot::outtakeRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command speakerCommand() {
    return Commands.runOnce(robot::prepareSpeakerRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command passCommand() {
    return Commands.runOnce(robot::preparePassRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command stowCommand() {
    return Commands.runOnce(robot::stowRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command idleWithGpCommand() {
    return Commands.runOnce(robot::idleWithGpRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_WITH_GP));
  }

  public Command stopIntakingCommand() {
    return Commands.runOnce(robot::stowRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command waitSubwooferCommand() {
    return Commands.runOnce(robot::waitSubwooferRequest, requirements);
  }

  public Command waitPodiumCommand() {
    return Commands.runOnce(robot::waitPodiumRequest, requirements);
  }

  public Command waitAmpCommand() {
    return Commands.runOnce(robot::waitAmpRequest, requirements);
  }

  public Command waitSpeakerCommand() {
    return Commands.runOnce(robot::waitSpeakerRequest, requirements);
  }

  public Command confirmShotCommand() {

    return Commands.runOnce(robot::confirmShotRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command ampCommand() {
    return Commands.runOnce(robot::prepareAmpRequest, requirements);
  }

  public Command stopShootingCommand() {
    return Commands.runOnce(robot::stopShootingRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_WITH_GP));
  }

  public Command feedingCommand() {
    return Commands.runOnce(robot::prepareFeedRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command waitFeedingCommand() {
    return Commands.runOnce(robot::waitFeedRequest, requirements);
  }

  public Command subwooferCommand() {
    return Commands.runOnce(robot::prepareSubwooferRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command podiumCommand() {
    return Commands.runOnce(robot::preparePodiumRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command climbUpCommand() {
    return Commands.runOnce(robot::nextClimbStateRequest, requirements);
  }

  public Command climbDownCommand() {
    return Commands.runOnce(robot::previousClimbStateRequest, requirements);
  }

  public Command unjamCommand() {
    return Commands.runOnce(robot::unjamRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP));
  }

  public Command redAutoIntake() {
    return Commands.sequence(
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(15.14, 6.02, Rotation2d.fromDegrees(49.1))),
                new AutoPoint(new Pose2d(15.629, 6.610, Rotation2d.fromDegrees(49.1))),
                new AutoPoint(new Pose2d(16.079, 7.219, Rotation2d.fromDegrees(51.7))))),
        Commands.print("Raise elevator, intake coral --> finished"),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(15.14, 6.801, Rotation2d.fromDegrees(51.7))))));
  }

  public Command blueAutoIntake() {
    return Commands.sequence(null);
  }
}
