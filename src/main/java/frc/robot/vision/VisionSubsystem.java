package frc.robot.vision;

import frc.robot.imu.ImuSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import java.util.List;
import java.util.ArrayList;

public class VisionSubsystem extends StateMachine<VisionState> {
  private final ImuSubsystem imu;
  private final Limelight leftLimelight;
  private final Limelight rightLimelight;
  private final List<VisionResult> processedVisionResult = new ArrayList<>();


  public VisionSubsystem(ImuSubsystem imu, Limelight leftLimelight, Limelight rightLimelight) {
    super(SubsystemPriority.VISION, VisionState.DEFAULT_STATE);
    this.imu = imu;
    this.leftLimelight = leftLimelight;
    this.rightLimelight = rightLimelight;
  }

  @Override
  protected void collectInputs() {
    var leftResult = leftLimelight.getRawVisionResult();
    var rightResult = rightLimelight.getRawVisionResult();
    processedVisionResult.clear();

    if (leftResult.isPresent()) {
      processedVisionResult.add(leftResult.get());
    }
    if(rightResult.isPresent()) {
      processedVisionResult.add(rightResult.get());
    }

  }

  public List<VisionResult> getVisionResult() {
    return processedVisionResult;
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
  }
}
