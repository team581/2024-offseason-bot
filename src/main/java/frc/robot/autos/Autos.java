package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class Autos extends LifecycleSubsystem {

  private static Command wrapAutoEvent(String commandName, Command command) {
    return Commands.sequence(
            Commands.print("[COMMANDS] Starting auto event " + commandName),
            command.deadlineWith(
                Commands.waitSeconds(5)
                    .andThen(
                        Commands.print(
                            "[COMMANDS] Auto event "
                                + commandName
                                + " has been running for 5+ seconds!"))),
            Commands.print("[COMMANDS] Finished auto event " + commandName))
        .handleInterrupt(() -> System.out.println("[COMMANDS] Cancelled auto event " + commandName))
        .withName(commandName);
  }

  private static void registerCommand(String eventName, Command command) {
    NamedCommands.registerCommand(eventName, wrapAutoEvent("Auto_" + eventName, command));
  }

  private final RobotCommands robotCommands;
  private final AutoChooser autoChooser;
  private final AutoCommands autoCommands;
  private final SwerveSubsystem swerve;
  private final LocalizationSubsystem localization;

  public Autos(
      RobotCommands robotCommands,
      RobotManager robotManager,
      SwerveSubsystem swerve,
      LocalizationSubsystem localization) {
    super(SubsystemPriority.AUTOS);

    this.robotCommands = robotCommands;
    this.swerve = swerve;
    this.localization = localization;

    autoCommands = new AutoCommands(robotCommands, robotManager);

    autoChooser = new AutoChooser(autoCommands);

    AutoBuilder.configureHolonomic(
        localization::getPose,
        localization::resetPose,
        swerve::getRobotRelativeSpeeds,
        (robotRelativeSpeeds) -> {
          swerve.setRobotRelativeAutoSpeeds(robotRelativeSpeeds);
        },
        new HolonomicPathFollowerConfig(
            new PIDConstants(4.0, 0.0, 0.0),
            new PIDConstants(2.5, 0.0, 0.0),
            4.4,
            0.387,
            new ReplanningConfig(true, true)),
        () -> false,
        swerve);

    registerCommand("speakerShot", autoCommands.speakerShotWithTimeout());
    registerCommand("intakeAssist", robotCommands.intakeAssistCommand());
    registerCommand("dynamic5Piece", autoCommands.dynamicAmp5PieceCommand());
    registerCommand("waitingSpeakerCommand", robotCommands.waitSpeakerCommand());
  }

  public Command getAutoCommand() {
    return autoChooser.getAutoCommand();
  }

  @Override
  public void disabledPeriodic() {
    // Constantly load the selected auto to avoid lag on auto init
    getAutoCommand();
  }
}
