package frc.robot.autos.amp;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.trailblazer.AutoPoint;
import frc.robot.autos.trailblazer.AutoSegment;
import frc.robot.autos.trailblazer.Trailblazer;
import frc.robot.robot_manager.RobotManager;

public class Op345Auto extends BaseAuto {
  public Op345Auto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Command getBlueAutoCommand() {
    return Commands.none();
  }

  @Override
  protected Command getRedAutoCommand() {
    return Commands.sequence(
        Commands.print("example command on auto start"),
        Commands.runOnce(
            () ->
                robotManager.localization.resetPose(
                    new Pose2d(15.78, 6.67, Rotation2d.fromDegrees(-60.48)))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(15.78, 6.67, Rotation2d.fromDegrees(-60.48))),
                new AutoPoint(new Pose2d(14.676, 6.767, Rotation2d.fromDegrees(-26.85))))),
        Commands.sequence(autoCommands.speakerShotWithTimeout(), actions.intakeAssistCommand()),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(13.68, 6.99, Rotation2d.fromDegrees(0.0))),
                new AutoPoint(new Pose2d(12.64, 7.06, Rotation2d.fromDegrees(-21.67))))),
        Commands.sequence(autoCommands.speakerShotWithTimeout(), actions.intakeAssistCommand()),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(8.52, 7.46, Rotation2d.fromDegrees(0.0))),
                new AutoPoint(new Pose2d(12.58, 6.01, Rotation2d.fromDegrees(-6.63))))),
        Commands.sequence(autoCommands.speakerShotWithTimeout(), actions.intakeAssistCommand()),
        trailblazer.followSegment(
            new AutoSegment(new AutoPoint(new Pose2d(10.71, 6.37, Rotation2d.fromDegrees(0.0))))));
  }
}
