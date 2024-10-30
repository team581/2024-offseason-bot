package frc.robot.swerve;

import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;

public class SnapUtil {

  public static double getAmpAngle() {
    return FmsSubsystem.isRedAlliance() ? 90 : (-90.0);
  }

  public static double getPodiumAngle() {
    // return whatever the amp angle is
    return FmsSubsystem.isRedAlliance() ? 0 : (180.0);
  }

  public static double getSubwooferAngle() {
    // return whatever the amp angle is
    return FmsSubsystem.isRedAlliance() ? 0 : (180.0);
  }

  public static double getClimbingAngle(ImuSubsystem imu) {
    if (FmsSubsystem.isRedAlliance()) {
      if (imu.getRobotHeading() < 0 && imu.getRobotHeading() < -120) {
        return -60;
      } else if (imu.getRobotHeading() > 0 && imu.getRobotHeading() < 120) {
        return 60;
      } else if (imu.getRobotHeading() > 120 && imu.getRobotHeading() < 240) {
        return 180;
      }
    }
    if (!FmsSubsystem.isRedAlliance()) {
      if (imu.getRobotHeading() > 60 && imu.getRobotHeading() < 180) {
        return 120;
      } else if (imu.getRobotHeading() > 180 && imu.getRobotHeading() < 300) {
        return 240;
      }
      // else if(imu.getRobotHeading().getDegrees()<-60&&imu.getRobotHeading().getDegrees()<60){
      else {
        return 0;
      }
    }
    return 0;
  }

  private SnapUtil() {}
}
