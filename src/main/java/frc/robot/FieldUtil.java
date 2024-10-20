package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.fms.FmsSubsystem;
import frc.robot.vision.VisionSubsystem;

public class FieldUtil {
      
    public static Pose2d getSpeakerPose() {
if (FmsSubsystem.isRedAlliance()) {
      return VisionSubsystem.ORIGINAL_RED_SPEAKER;
    } else {
      return VisionSubsystem.ORIGINAL_BLUE_SPEAKER;
    }
    }
}
