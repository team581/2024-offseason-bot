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
                    new Pose2d(15.18, 6.41, Rotation2d.fromDegrees(0.0)))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(12.00, 5, Rotation2d.fromDegrees(-10))),
                new AutoPoint(new Pose2d(10.00, 5, Rotation2d.fromDegrees(10))),
                new AutoPoint(new Pose2d(8.00, 5, Rotation2d.fromDegrees(10))),
                new AutoPoint(new Pose2d(7.00, 3.8, Rotation2d.fromDegrees(-10))),
                new AutoPoint(new Pose2d(5.00, 4.8, Rotation2d.fromDegrees(10))),
                new AutoPoint(new Pose2d(4.00, 3.28, Rotation2d.fromDegrees(-10))))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(7.0, 7.0, Rotation2d.fromDegrees(135.0))),
                new AutoPoint(new Pose2d(12.0, 7.0, Rotation2d.fromDegrees(45.0))),
                new AutoPoint(new Pose2d(12.0, 2.0, Rotation2d.fromDegrees(-45.0))),
                new AutoPoint(new Pose2d(7.0, 2.0, Rotation2d.fromDegrees(-135.0))),
                new AutoPoint(new Pose2d(8.0, 6.0, Rotation2d.fromDegrees(135.0))),
                new AutoPoint(new Pose2d(11.0, 6.0, Rotation2d.fromDegrees(45.0))),
                new AutoPoint(new Pose2d(11.0, 3.0, Rotation2d.fromDegrees(-45.0))),
                new AutoPoint(new Pose2d(8.0, 3.0, Rotation2d.fromDegrees(-135.0))),
                new AutoPoint(new Pose2d(9.0, 5.0, Rotation2d.fromDegrees(135.0))),
                new AutoPoint(new Pose2d(10.0, 5.0, Rotation2d.fromDegrees(45.0))),
                new AutoPoint(new Pose2d(10.0, 4.0, Rotation2d.fromDegrees(-45.0))),
                new AutoPoint(new Pose2d(9.0, 4.0, Rotation2d.fromDegrees(-135.0))))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(9.98, 6.55, Rotation2d.fromDegrees(-5.0))),
                new AutoPoint(new Pose2d(8.60, 5.85, Rotation2d.fromDegrees(100.0))))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(10.78, 7.34, Rotation2d.fromDegrees(-80.0))),
                new AutoPoint(new Pose2d(13.3, 6.59, Rotation2d.fromDegrees(-100))))));
  }
}
