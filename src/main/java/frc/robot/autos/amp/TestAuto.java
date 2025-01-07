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
                    new Pose2d(9.0, 4.0, Rotation2d.fromDegrees(0)))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoConstraintOptions(false, 5, 500, 8.0, 5000),
                new AutoPoint(new Pose2d(11.8, 4.0, Rotation2d.fromDegrees(0))))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoConstraintOptions(false, 5, 500, 8.0, 500),
                new AutoPoint(new Pose2d(10.9, 5.0, Rotation2d.fromDegrees(-10))),
                new AutoPoint(new Pose2d(16.157, 7.043, Rotation2d.fromDegrees(-125.216))))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoConstraintOptions(false, 5, 500, 8.0, 500),
                new AutoPoint(new Pose2d(13.6, 5.2, Rotation2d.fromDegrees(-120.559))))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoConstraintOptions(false, 5, 500, 8.0, 500),
                new AutoPoint(new Pose2d(16.157, 7.043, Rotation2d.fromDegrees(-125.216))))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoConstraintOptions(false, 5, 500, 8.0, 500),
                new AutoPoint(new Pose2d(13.6, 5.2, Rotation2d.fromDegrees(-120.559))))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoConstraintOptions(false, 5, 500, 8.0, 500),
                new AutoPoint(new Pose2d(16.157, 7.043, Rotation2d.fromDegrees(-125.216))))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoConstraintOptions(false, 5, 500, 8.0, 500),
                new AutoPoint(new Pose2d(12.2, 5.5, Rotation2d.fromDegrees(-60.0))),
                new AutoPoint(new Pose2d(12.46, 5.16, Rotation2d.fromDegrees(-59.927))))));
  }
}
