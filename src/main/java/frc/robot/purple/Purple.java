package frc.robot.purple;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.vision.LimelightHelpers;

public class Purple {
  private static final double X_ADJUST_P = 0.1;

  public static ChassisSpeeds getPurpleAdjustmentRobotRelative() {
    var tx = LimelightHelpers.getTX("asdasd");
    

    if (tx == 0) {
      return new ChassisSpeeds();
    }

    var error = getXError(tx);

    var adjustment = getRobotAdjustmentRobotRelative(error);

    return adjustment;
  }

  private static double getXError(double tx) {
    return -tx;
  }

  private static ChassisSpeeds getRobotAdjustmentRobotRelative(double xError) {
    // adjust side to side, don't care about forward/backward, don't care about rotation
    double xTranslation = X_ADJUST_P * xError;
    ChassisSpeeds translation = new ChassisSpeeds(xTranslation, 0, 0);
    return translation;
  }
}
