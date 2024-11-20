package frc.robot.autos.red.amp;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.trailblazer.AutoPoint;
import frc.robot.autos.trailblazer.AutoSegment;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.autos.trailblazer.Trailblazer;
import frc.robot.robot_manager.RobotManager;

public class RedOp345Auto extends BaseAuto {
  public RedOp345Auto(RobotManager robotManager, Trailblazer trailblazer, RobotCommands actions) {
    super(robotManager, trailblazer);
  }

  public Command getAutoCommand() {
    return Commands.sequence(
        Commands.print("example command on auto start"),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(15.811, 6.713, new Rotation2d(-170.80))),
                new AutoPoint(new Pose2d(14.676, 6.767, new Rotation2d(-26.85)), Commands.sequence(autoCommands.speakerShotWithTimeout(), actions.intakeAssistCommand())),
                new AutoPoint(new Pose2d(13.959, 6.872, new Rotation2d())),
                new AutoPoint(new Pose2d(12.64, 7.063, new Rotation2d()), Commands.sequence(autoCommands.speakerShotWithTimeout(), actions.intakeAssistCommand())),
                new AutoPoint(new Pose2d(8.605, 7.449, new Rotation2d(0.0))),
                new AutoPoint(new Pose2d(12.64, 7.063, new Rotation2d(-8.0)), autoCommands.speakerShotWithTimeout()),
                new AutoPoint(new Pose2d(11.033, 6.879, new Rotation2d()), actions.intakeAssistCommand()),
                new AutoPoint(new Pose2d(10.827, 6.42, new Rotation2d())),
                new AutoPoint(new Pose2d(12.553, 6.036, new Rotation2d(-8.0)), autoCommands.speakerShotWithTimeout()))));
  }
}
