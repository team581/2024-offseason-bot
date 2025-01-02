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
                    new Pose2d(15.0, 5.65, Rotation2d.fromDegrees(0.0)))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(11.672, 5.638, Rotation2d.fromDegrees(0.0))),
                new AutoPoint(new Pose2d(11.672, 6.638, Rotation2d.fromDegrees(0.0))),
                new AutoPoint(new Pose2d(10.3, 5.746, Rotation2d.fromDegrees(0))),
                new AutoPoint(new Pose2d(10.30, 3.8, Rotation2d.fromDegrees(0))))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(12.175, 4.437, Rotation2d.fromDegrees(10))),
                new AutoPoint(new Pose2d(11.338, 5.035, Rotation2d.fromDegrees(-20))),
                new AutoPoint(new Pose2d(12.263, 5.202, Rotation2d.fromDegrees(5))),
                new AutoPoint(new Pose2d(11.5, 5.4, Rotation2d.fromDegrees(20))),
                new AutoPoint(new Pose2d(12.2, 5.6, Rotation2d.fromDegrees(-10))),
                new AutoPoint(new Pose2d(13.15, 6.737, Rotation2d.fromDegrees(30))))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(10.3, 4.136, Rotation2d.fromDegrees(180))))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(13.0, 4.136, Rotation2d.fromDegrees(-90))),
                new AutoPoint(new Pose2d(15.0, 5.65, Rotation2d.fromDegrees(0))))));
  }
}
