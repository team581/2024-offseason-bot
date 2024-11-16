package frc.robot.autos.trailblazer;

import frc.robot.autos.trailblazer.constraints.AutoPointConstraint;
import java.util.List;

public class AutoSegment {
  public final List<AutoPoint> points;
  public final List<AutoPointConstraint> globalConstraints;

  public AutoSegment(List<AutoPointConstraint> globalConstraints, List<AutoPoint> points) {
    this.globalConstraints = globalConstraints;
    this.points = points;
  }

  public AutoSegment(List<AutoPointConstraint> globalConstraints, AutoPoint... points) {
    this(globalConstraints, List.of(points));
  }

  public AutoSegment(List<AutoPoint> points) {
    this(List.of(), points);
  }

  public AutoSegment(AutoPoint... points) {
    this(List.of(), points);
  }
}
