package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class Autos extends LifecycleSubsystem {

  private final AutoChooser autoChooser;
  private final AutoCommands autoCommands;

  public Autos() {
    super(SubsystemPriority.AUTOS);

    autoCommands = new AutoCommands();

    autoChooser = new AutoChooser(autoCommands);
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
