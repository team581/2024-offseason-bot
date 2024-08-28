package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmAngle {
  public static final Rotation2d SUBWOOFER =
      Rotation2d.fromDegrees(25); // NONE OF THESE ACCURATE/TUNED
  public static final Rotation2d DROP = Rotation2d.fromDegrees(5);
  public static final Rotation2d PODIUM = Rotation2d.fromDegrees(20);
  public static final Rotation2d IDLE = Rotation2d.fromDegrees(0);
  public static final Rotation2d CLIMBING = Rotation2d.fromDegrees(30);
  // TODO: Rename climbing to be CLIMBING_1_LINEUP, and add a new CLIMBING_2_HANGING
  // TODO: add amping??
}
