package frc.robot.swerve;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class SwerveSubsystem extends StateMachine<SwerveState>{
  public SwerveSubsystem(CommandXboxController driveController){
    super(SubsystemPriority.SWERVE, SwerveState.TELEOP);
  }

}
