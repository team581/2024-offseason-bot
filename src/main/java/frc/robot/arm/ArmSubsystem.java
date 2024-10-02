package frc.robot.arm;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ArmSubsystem extends StateMachine<ArmState> {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private double leftMotorAngle;
  private double rightMotorAngle;
  private double distanceToSpeaker;
  private double distanceToFeedSpot;
  private double lowestSeenAngleLeft = Double.MAX_VALUE;
  private double lowestSeenAngleRight = Double.MAX_VALUE;
  private InterpolatingDoubleTreeMap speakerDistanceToAngle = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap feedSpotDistanceToAngle = new InterpolatingDoubleTreeMap();
  private final PositionVoltage positionRequest =
      new PositionVoltage(0)
          .withEnableFOC(false)
          .withLimitReverseMotion(false)
          .withOverrideBrakeDurNeutral(true);

  //  private final StaticBrake staticBrake =
  //      new StaticBrake();
  // IF withOverrideBrakeDurNeutral DOEST WORK

  public void setDistanceToSpeaker(double distance) {
    distanceToSpeaker = distance;
  }

  public void setDistanceToFeedSpot(double distance) {
    distanceToFeedSpot = distance;
  }

  public ArmSubsystem(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.ARM, ArmState.IDLE);
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    leftMotor.getConfigurator().apply(RobotConfig.get().arm().leftMotorConfig());
    rightMotor.getConfigurator().apply(RobotConfig.get().arm().rightMotorConfig());
    RobotConfig.get().arm().feedSpotDistanceToAngle().accept(feedSpotDistanceToAngle);
    RobotConfig.get().arm().speakerDistanceToAngle().accept(speakerDistanceToAngle);
  }

  public void setState(ArmState newState) {
    if (getState() != ArmState.PRE_MATCH_HOMING) {

      setStateFromRequest(newState);
    }
  }

  // public void disabledPeriodic()
  // {
  //   rightMotor.setControl(staticBrake);
  //   leftMotor.setControl(staticBrake);
  // }
  // IF withOverrideBrakeDurNeutral DOESNT WORK
  public boolean atGoal() {
    return switch (getState()) {
      case IDLE, PRE_MATCH_HOMING -> true;
      case SPEAKER_SHOT ->
          MathUtil.isNear(speakerDistanceToAngle.get(distanceToSpeaker), leftMotorAngle, 1)
              && MathUtil.isNear(speakerDistanceToAngle.get(distanceToSpeaker), rightMotorAngle, 1);
      case DROP ->
          MathUtil.isNear(ArmAngle.DROP.getDegrees(), leftMotorAngle, 1)
              && MathUtil.isNear(ArmAngle.DROP.getDegrees(), rightMotorAngle, 1);
      case SUBWOOFER_SHOT ->
          MathUtil.isNear(ArmAngle.SUBWOOFER.getDegrees(), leftMotorAngle, 1)
              && MathUtil.isNear(ArmAngle.SUBWOOFER.getDegrees(), rightMotorAngle, 1);
      case FEEDING ->
          MathUtil.isNear(feedSpotDistanceToAngle.get(distanceToFeedSpot), leftMotorAngle, 1)
              && MathUtil.isNear(
                  feedSpotDistanceToAngle.get(distanceToFeedSpot), rightMotorAngle, 1);
      case PODIUM_SHOT ->
          MathUtil.isNear(ArmAngle.PODIUM.getDegrees(), leftMotorAngle, 1)
              && MathUtil.isNear(ArmAngle.PODIUM.getDegrees(), rightMotorAngle, 1);
      case CLIMBING_1_LINEUP ->
          MathUtil.isNear(ArmAngle.CLIMBING_1_LINEUP.getDegrees(), leftMotorAngle, 1)
              && MathUtil.isNear(ArmAngle.CLIMBING_1_LINEUP.getDegrees(), rightMotorAngle, 1);
      case CLIMBING_2_HANGING ->
          MathUtil.isNear(ArmAngle.CLIMBING_2_HANGING.getDegrees(), leftMotorAngle, 1)
              && MathUtil.isNear(ArmAngle.CLIMBING_2_HANGING.getDegrees(), rightMotorAngle, 1);

      case AMP ->
          MathUtil.isNear(ArmAngle.AMP.getDegrees(), leftMotorAngle, 1)
              && MathUtil.isNear(ArmAngle.AMP.getDegrees(), rightMotorAngle, 1);
      case PASS ->
          MathUtil.isNear(ArmAngle.PASS.getDegrees(), leftMotorAngle, 1)
              && MathUtil.isNear(ArmAngle.PASS.getDegrees(), rightMotorAngle, 1);
    };
  }

  @Override
  protected void collectInputs() {
    leftMotorAngle = Units.rotationsToDegrees(leftMotor.getPosition().getValueAsDouble());
    rightMotorAngle = Units.rotationsToDegrees(rightMotor.getPosition().getValueAsDouble());
    if (DriverStation.isDisabled()) {
      lowestSeenAngleLeft = Math.min(lowestSeenAngleLeft, leftMotorAngle);
      lowestSeenAngleRight = Math.min(lowestSeenAngleRight, rightMotorAngle);
    }
  }

  @Override
  protected void afterTransition(ArmState newState) {
    switch (newState) {
      case PRE_MATCH_HOMING, IDLE -> {
        leftMotor.disable();
        rightMotor.disable();
      }

      case CLIMBING_1_LINEUP -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clamp(ArmAngle.CLIMBING_1_LINEUP.getDegrees()))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clamp(ArmAngle.CLIMBING_1_LINEUP.getDegrees()))));
      }
      case CLIMBING_2_HANGING -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clamp(ArmAngle.CLIMBING_2_HANGING.getDegrees()))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clamp(ArmAngle.CLIMBING_2_HANGING.getDegrees()))));
      }

      case DROP -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clamp(ArmAngle.DROP.getDegrees()))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clamp(ArmAngle.DROP.getDegrees()))));
      }

      case PODIUM_SHOT -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clamp(ArmAngle.PODIUM.getDegrees()))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clamp(ArmAngle.PODIUM.getDegrees()))));
      }
      case SUBWOOFER_SHOT -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clamp(ArmAngle.SUBWOOFER.getDegrees()))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clamp(ArmAngle.SUBWOOFER.getDegrees()))));
      }

      case FEEDING -> {
        double newAngle =
            Units.degreesToRotations(clamp(feedSpotDistanceToAngle.get(distanceToFeedSpot)));
        leftMotor.setControl(positionRequest.withPosition(newAngle));
        rightMotor.setControl(positionRequest.withPosition(newAngle));
      }
      case SPEAKER_SHOT -> {
        var newAngle =
            Units.degreesToRotations(clamp(speakerDistanceToAngle.get(distanceToSpeaker)));

        leftMotor.setControl(positionRequest.withPosition(newAngle));
        rightMotor.setControl(positionRequest.withPosition(newAngle));
      }
      case AMP -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clamp(ArmAngle.AMP.getDegrees()))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clamp(ArmAngle.AMP.getDegrees()))));
      }
      case PASS -> {
        leftMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clamp(ArmAngle.PASS.getDegrees()))));
        rightMotor.setControl(
            positionRequest.withPosition(
                Units.degreesToRotations(clamp(ArmAngle.PASS.getDegrees()))));
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    switch (getState()) {
      case SPEAKER_SHOT -> {
        var newAngle =
            Units.degreesToRotations(clamp(speakerDistanceToAngle.get(distanceToSpeaker)));
        leftMotor.setControl(positionRequest.withPosition(newAngle));
        rightMotor.setControl(positionRequest.withPosition(newAngle));
      }
      case FEEDING -> {
        double newAngle =
            Units.degreesToRotations(clamp(feedSpotDistanceToAngle.get(distanceToFeedSpot)));
        leftMotor.setControl(positionRequest.withPosition(newAngle));
        rightMotor.setControl(positionRequest.withPosition(newAngle));
      }
      default -> {}
    }

    DogLog.log("Arm/Left/StatorCurrent", leftMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Arm/Left/SupplyCurrent", leftMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Arm/Left/ArmAngle", leftMotor.getPosition().getValueAsDouble());
    DogLog.log("Arm/Left/AppliedVoltage", leftMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Arm/Left/LowestSeenAngle", lowestSeenAngleLeft);

    DogLog.log("Arm/Right/StatorCurrent", rightMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Arm/Right/SupplyCurrent", rightMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Arm/Right/ArmAngle", rightMotor.getPosition().getValueAsDouble());
    DogLog.log("Arm/Right/AppliedVoltage", rightMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Arm/Right/LowestSeenAngle", lowestSeenAngleRight);

    if (DriverStation.isEnabled() && getState() == ArmState.PRE_MATCH_HOMING) {
      // We are enabled and still in pre match homing
      // Reset the motor positions, and then transition to idle state
      leftMotor.setPosition(
          Units.degreesToRotations(
              RobotConfig.get().arm().minAngle() + (leftMotorAngle - lowestSeenAngleLeft)));
      rightMotor.setPosition(
          Units.degreesToRotations(
              RobotConfig.get().arm().minAngle() + (rightMotorAngle - lowestSeenAngleRight)));
      setStateFromRequest(ArmState.IDLE);
    }
  }

  private static double clamp(double armAngle) {
    return MathUtil.clamp(
        armAngle, RobotConfig.get().arm().minAngle(), RobotConfig.get().arm().maxAngle());
  }
}
