package frc.robot.autos.trailblazer;

import frc.robot.autos.trailblazer.constraints.AutoPointConstraint;
import java.util.List;

/**
 * A segment is a path (a continuous set of {@link AutoPoint points}) that the roobt will follow.
 */
public class AutoSegment {
  public final List<AutoPoint> points;

  /**
   * Constraints to apply to any points that don't have their own constraints specified. If a point
   * specifies its own constraints, this field will be ignored.
   */
  public final List<AutoPointConstraint> defaultConstraints;

  public AutoSegment(List<AutoPointConstraint> defaultConstraints, List<AutoPoint> points) {
    this.defaultConstraints = defaultConstraints;
    this.points = points;
  }

  public AutoSegment(List<AutoPointConstraint> defaultConstraints, AutoPoint... points) {
    this(defaultConstraints, List.of(points));
  }

  public AutoSegment(List<AutoPoint> points) {
    this(List.of(), points);
  }

  public AutoSegment(AutoPoint... points) {
    this(List.of(), points);
  }
}
