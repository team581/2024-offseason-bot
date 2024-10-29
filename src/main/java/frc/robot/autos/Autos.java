package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
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

  public Autos(RobotCommands robotCommands, RobotManager robotManager) {
    super(SubsystemPriority.AUTOS);

    this.robotCommands = robotCommands;

    autoCommands = new AutoCommands(robotCommands, robotManager);

    autoChooser = new AutoChooser(autoCommands);

    registerCommand("speakerShot", robotCommands.speakerCommand());
    registerCommand("intakeAssist", robotCommands.intakeAssistCommand());
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
