package frc.robot.autos.trailblazer.constraints;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Objects;

/**
 * Temporary class until we update to 2025 WPILib and can use this PR
 * https://github.com/wpilibsuite/allwpilib/pull/6414.
 */
class ChassisSpeedsComparable extends ChassisSpeeds {
  private static void round(ChassisSpeeds speeds) {
    speeds.vxMetersPerSecond = Math.round(speeds.vxMetersPerSecond * 100.0) / 100.0;
    speeds.vyMetersPerSecond = Math.round(speeds.vyMetersPerSecond * 100.0) / 100.0;
    speeds.omegaRadiansPerSecond = Math.round(speeds.omegaRadiansPerSecond * 100.0) / 100.0;
  }

  public ChassisSpeedsComparable(ChassisSpeeds speeds) {
    super(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    round(this);
  }

  @Override
  public boolean equals(Object obj) {
    if (obj == this) {
      return true;
    }

    if (obj instanceof ChassisSpeedsComparable) {
      ChassisSpeedsComparable other = (ChassisSpeedsComparable) obj;
      return other.vxMetersPerSecond == vxMetersPerSecond
          && other.vyMetersPerSecond == vyMetersPerSecond
          && other.omegaRadiansPerSecond == omegaRadiansPerSecond;
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
  }
}
