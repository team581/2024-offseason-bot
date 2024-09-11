package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmAngle {
  public static final Rotation2d SUBWOOFER =
      Rotation2d.fromDegrees(25); // NONE OF THESE ACCURATE/TUNED
  public static final Rotation2d DROP = Rotation2d.fromDegrees(5);
  public static final Rotation2d PODIUM = Rotation2d.fromDegrees(20);
  public static final Rotation2d IDLE = Rotation2d.fromDegrees(0);
  public static final Rotation2d CLIMBING_1_LINEUP = Rotation2d.fromDegrees(30);
  public static final Rotation2d CLIMBING_2_HANGING = Rotation2d.fromDegrees(20);
  public static final Rotation2d AMP = Rotation2d.fromDegrees(20);
  public static final Rotation2d PASS = Rotation2d.fromDegrees(5);
}
