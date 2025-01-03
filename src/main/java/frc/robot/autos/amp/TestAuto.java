package frc.robot.autos.amp;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.trailblazer.AutoPoint;
import frc.robot.autos.trailblazer.AutoSegment;
import frc.robot.autos.trailblazer.Trailblazer;
import frc.robot.autos.trailblazer.constraints.AutoConstraintOptions;
import frc.robot.robot_manager.RobotManager;

public class TestAuto extends BaseAuto {
  public TestAuto(RobotManager robotManager, Trailblazer trailblazer) {
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
                    new Pose2d(15, 7, Rotation2d.fromDegrees(0.0)))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(
                    new Pose2d(10, 7, Rotation2d.fromDegrees(0)),
                    new AutoConstraintOptions(false, 100, 100, 7, 500)),
                new AutoPoint(
                    new Pose2d(10, 5, Rotation2d.fromDegrees(0)),
                    new AutoConstraintOptions(false, 500, 500, 7, 500)),
                new AutoPoint(
                    new Pose2d(14, 5, Rotation2d.fromDegrees(0)),
                    new AutoConstraintOptions(false, 500, 500, 7, 500)))));
  }
}
