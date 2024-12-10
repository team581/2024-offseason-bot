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
                new AutoPoint(new Pose2d(12.00, 6.28, Rotation2d.fromDegrees(0.0))),
                new AutoPoint(new Pose2d(10.63, 7.42, Rotation2d.fromDegrees(
                    0.0))),

                    new AutoPoint(new Pose2d(8.63, 7.42, Rotation2d.fromDegrees(0.0))))

                    ),

        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(11.82, 6.59, Rotation2d.fromDegrees(-12.24))))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(9.98, 6.55, Rotation2d.fromDegrees(14.02))),
                new AutoPoint(new Pose2d(8.60, 5.85, Rotation2d.fromDegrees(14.02))))),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(10.78, 7.34, Rotation2d.fromDegrees(0))),
                new AutoPoint(new Pose2d(11.82, 6.59, Rotation2d.fromDegrees(-12.24))))));
  }
}
