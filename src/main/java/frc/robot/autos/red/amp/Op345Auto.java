package frc.robot.autos.red.amp;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.trailblazer.AutoPoint;
import frc.robot.autos.trailblazer.AutoSegment;
import frc.robot.robot_manager.RobotManager;

public class Op345Auto extends BaseAuto {
  public Op345Auto(RobotManager robotManager) {
    super(robotManager);
  }

  public Command getAutoCommand() {
    return Commands.sequence(
        Commands.print("example command on auto start"),
        trailblazer.followSegment(
            new AutoSegment(new AutoPoint(new Pose2d(2, 4, new Rotation2d())))));
  }
}
