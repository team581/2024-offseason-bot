package frc.robot.swerve;

import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import java.util.List;

public class SnapUtil {

  public static double getAmpAngle() {
    return FmsSubsystem.isRedAlliance() ? 90 : (90.0);
  }

  public static double getPodiumAngle() {
    // return whatever the amp angle is
    return FmsSubsystem.isRedAlliance() ? 0 : (180.0);
  }

  public static double getSubwooferAngle() {
    // return whatever the amp angle is
    return FmsSubsystem.isRedAlliance() ? 0 : (180.0);
  }

  public static double getReef(int reefNum) {
    switch (reefNum) {
      case 1 -> {
        return getReefAngle1();
      }
      case 2 -> {
        return getReefAngle2();
      }
      case 3 -> {
        return getReefAngle3();
      }
      case 4 -> {
        return getReefAngle4();
      }
      case 5 -> {
        return getReefAngle5();
      }
      case 6 -> {
        return getReefAngle6();
      }
      default -> {
        return 0.0;
      }
    }
  }

  public static double getReefAngle1() {
    // return whatever the amp angle is
    return FmsSubsystem.isRedAlliance() ? 0.0 : (180.0);
  }

  public static double getReefAngle2() {
    // return whatever the amp angle is
    return FmsSubsystem.isRedAlliance() ? 60.0 : (240.0);
  }

  public static double getReefAngle3() {
    // return whatever the amp angle is
    return FmsSubsystem.isRedAlliance() ? 120.0 : (300.0);
  }

  public static double getReefAngle4() {
    // return whatever the amp angle is
    return FmsSubsystem.isRedAlliance() ? 180.0 : (0.0);
  }

  public static double getReefAngle5() {
    // return whatever the amp angle is
    return FmsSubsystem.isRedAlliance() ? 240.0 : (60.0);
  }

  public static double getReefAngle6() {
    // return whatever the amp angle is
    return FmsSubsystem.isRedAlliance() ? 300.0 : (120.0);
  }

  private static final List<Double> RED_STAGE_ANGLES = List.of(0.0, 120.0, -120.0);
  private static final List<Double> BLUE_STAGE_ANGLES =
      List.of(0.0 + 180.0, 120.0 - 180.0, -120 + 180.0);

  public static double getClimbingAngle(ImuSubsystem imu) {
    var usedAngles = FmsSubsystem.isRedAlliance() ? RED_STAGE_ANGLES : BLUE_STAGE_ANGLES;
    if (FmsSubsystem.isRedAlliance()) {
      var currentAngle = imu.getRobotHeading();

      var closestAngle = RED_STAGE_ANGLES.get(0);
      var smallestDifference = Double.POSITIVE_INFINITY;
      for (var angle : usedAngles) {
        if (Math.abs(angle - currentAngle) < smallestDifference) {
          closestAngle = angle;
          smallestDifference = Math.abs(angle - currentAngle);
        }
      }

      return closestAngle;
    } else {
      var currentAngle = imu.getRobotHeading();

      var closestAngle = BLUE_STAGE_ANGLES.get(0);
      var smallestDifference = Double.POSITIVE_INFINITY;
      for (var angle : usedAngles) {
        if (Math.abs(angle - currentAngle) < smallestDifference) {
          closestAngle = angle;
          smallestDifference = Math.abs(angle - currentAngle);
        }
      }

      return closestAngle;
    }
  }

  private SnapUtil() {}
}
