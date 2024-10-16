package frc.robot.vision;

import frc.robot.imu.ImuSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import java.util.ArrayList;
import java.util.List;

import dev.doglog.DogLog;

public class VisionSubsystem extends StateMachine<VisionState> {
  private final ImuSubsystem imu;
  private final Limelight leftLimelight;
  private final Limelight rightLimelight;
  private final List<VisionResult> interpolatedVisionResult = new ArrayList<>();

  public VisionSubsystem(ImuSubsystem imu, Limelight leftLimelight, Limelight rightLimelight) {
    super(SubsystemPriority.VISION, VisionState.DEFAULT_STATE);
    this.imu = imu;
    this.leftLimelight = leftLimelight;
    this.rightLimelight = rightLimelight;
  }

  @Override
  protected void collectInputs() {
    var leftInterpolatedVisionResult = leftLimelight.getInterpolatedVisionResult();
    var rightInterpolatedVisionResult = rightLimelight.getInterpolatedVisionResult();



    interpolatedVisionResult.clear();

    if (leftInterpolatedVisionResult.isPresent()) {
      interpolatedVisionResult.add(leftInterpolatedVisionResult.get());
    }
    if (rightInterpolatedVisionResult.isPresent()) {
      interpolatedVisionResult.add(rightInterpolatedVisionResult.get());
    }
  }

  public List<VisionResult> getInterpolatedVisionResult() {
    return interpolatedVisionResult;
  }
}
