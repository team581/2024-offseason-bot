package frc.robot.autos.trailblazer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.trailblazer.constraints.AutoPointConstraint;
import java.util.List;
import java.util.function.Supplier;

public class AutoPoint {
  public final Supplier<Pose2d> poseSupplier;
  public final List<AutoPointConstraint> constraints;
  public final Command command;

  public AutoPoint(
      Supplier<Pose2d> poseSupplier, Command command, List<AutoPointConstraint> constraints) {
    this.poseSupplier = poseSupplier;
    this.command = command;
    this.constraints = constraints;
  }

  public AutoPoint(Supplier<Pose2d> poseSupplier, Command command) {
    this(poseSupplier, command, List.of());
  }

  public AutoPoint(Supplier<Pose2d> poseSupplier, List<AutoPointConstraint> constraints) {
    this(poseSupplier, Commands.none(), constraints);
  }

  public AutoPoint(Supplier<Pose2d> poseSupplier) {
    this(poseSupplier, Commands.none(), List.of());
  }

  public AutoPoint(Pose2d pose, Command command, List<AutoPointConstraint> constraints) {
    this(() -> pose, command, constraints);
  }

  public AutoPoint(Pose2d pose, Command command) {
    this(pose, command, List.of());
  }

  public AutoPoint(Pose2d pose, List<AutoPointConstraint> constraints) {
    this(pose, Commands.none(), constraints);
  }

  public AutoPoint(Pose2d pose) {
    this(pose, Commands.none(), List.of());
  }
}
