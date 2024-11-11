package frc.robot.arm;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withEnableFOC(false).withOverrideBrakeDurNeutral(true);
  private final PositionVoltage pidRequest =
      new PositionVoltage(0).withEnableFOC(false).withOverrideBrakeDurNeutral(true);

  public void setDistanceToSpeaker(double distance) {
    distanceToSpeaker = distance;
  }

  public void setDistanceToFeedSpot(double distance) {
    distanceToFeedSpot = distance;
  }

  public ArmSubsystem(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.ARM, ArmState.PRE_MATCH_HOMING);
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

  public boolean atGoal() {
    return switch (getState()) {
      case PRE_MATCH_HOMING -> true;
      case IDLE ->
          MathUtil.isNear(ArmAngle.IDLE, leftMotorAngle, 1)
              && MathUtil.isNear(ArmAngle.IDLE, rightMotorAngle, 1);
      case SPEAKER_SHOT ->
          MathUtil.isNear(speakerDistanceToAngle.get(distanceToSpeaker), leftMotorAngle, 1)
              && MathUtil.isNear(speakerDistanceToAngle.get(distanceToSpeaker), rightMotorAngle, 1);
      case DROP ->
          MathUtil.isNear(ArmAngle.DROP, leftMotorAngle, 1)
              && MathUtil.isNear(ArmAngle.DROP, rightMotorAngle, 1);
      case SUBWOOFER_SHOT ->
          MathUtil.isNear(ArmAngle.SUBWOOFER, leftMotorAngle, 1)
              && MathUtil.isNear(ArmAngle.SUBWOOFER, rightMotorAngle, 1);
      case FEEDING ->
          MathUtil.isNear(feedSpotDistanceToAngle.get(distanceToFeedSpot), leftMotorAngle, 1)
              && MathUtil.isNear(
                  feedSpotDistanceToAngle.get(distanceToFeedSpot), rightMotorAngle, 1);
      case PODIUM_SHOT ->
          MathUtil.isNear(ArmAngle.PODIUM, leftMotorAngle, 1)
              && MathUtil.isNear(ArmAngle.PODIUM, rightMotorAngle, 1);
      case CLIMBING_1_LINEUP ->
          MathUtil.isNear(ArmAngle.CLIMBING_1_LINEUP, leftMotorAngle, 1)
              && MathUtil.isNear(ArmAngle.CLIMBING_1_LINEUP, rightMotorAngle, 1);
      case CLIMBING_2_HANGING ->
          MathUtil.isNear(ArmAngle.CLIMBING_2_HANGING, leftMotorAngle, 1)
              && MathUtil.isNear(ArmAngle.CLIMBING_2_HANGING, rightMotorAngle, 1);

      case AMP ->
          MathUtil.isNear(ArmAngle.AMP, leftMotorAngle, 1)
              && MathUtil.isNear(ArmAngle.AMP, rightMotorAngle, 1);
      case UNJAM ->
          MathUtil.isNear(ArmAngle.UNJAM, leftMotorAngle, 1)
              && MathUtil.isNear(ArmAngle.UNJAM, rightMotorAngle, 1);
      case PASS ->
          MathUtil.isNear(ArmAngle.PASS, leftMotorAngle, 1)
              && MathUtil.isNear(ArmAngle.PASS, rightMotorAngle, 1);
      case HOLD_FOR_INTAKE -> true;
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
      case PRE_MATCH_HOMING -> {
        leftMotor.disable();
        rightMotor.disable();
      }
      case IDLE -> {
        leftMotor.setControl(
            motionMagicRequest.withPosition(Units.degreesToRotations(clamp(ArmAngle.IDLE))));
        rightMotor.setControl(
            motionMagicRequest.withPosition(Units.degreesToRotations(clamp(ArmAngle.IDLE))));
      }
      case CLIMBING_1_LINEUP -> {
        leftMotor.setControl(
            motionMagicRequest.withPosition(
                Units.degreesToRotations(clamp(ArmAngle.CLIMBING_1_LINEUP))));
        rightMotor.setControl(
            motionMagicRequest.withPosition(
                Units.degreesToRotations(clamp(ArmAngle.CLIMBING_1_LINEUP))));
      }
      case CLIMBING_2_HANGING -> {
        leftMotor.setControl(
            motionMagicRequest.withPosition(
                Units.degreesToRotations(clamp(ArmAngle.CLIMBING_2_HANGING))));
        rightMotor.setControl(
            motionMagicRequest.withPosition(
                Units.degreesToRotations(clamp(ArmAngle.CLIMBING_2_HANGING))));
      }

      case DROP -> {
        leftMotor.setControl(
            pidRequest.withPosition(Units.degreesToRotations(clamp(ArmAngle.DROP))));
        rightMotor.setControl(
            pidRequest.withPosition(Units.degreesToRotations(clamp(ArmAngle.DROP))));
      }

      case PODIUM_SHOT -> {
        leftMotor.setControl(
            pidRequest.withPosition(Units.degreesToRotations(clamp(ArmAngle.PODIUM))));
        rightMotor.setControl(
            pidRequest.withPosition(Units.degreesToRotations(clamp(ArmAngle.PODIUM))));
      }
      case SUBWOOFER_SHOT -> {
        leftMotor.setControl(
            pidRequest.withPosition(Units.degreesToRotations(clamp(ArmAngle.SUBWOOFER))));
        rightMotor.setControl(
            pidRequest.withPosition(Units.degreesToRotations(clamp(ArmAngle.SUBWOOFER))));
      }

      case AMP -> {
        leftMotor.setControl(
            motionMagicRequest.withPosition(Units.degreesToRotations(clamp(ArmAngle.AMP))));
        rightMotor.setControl(
            motionMagicRequest.withPosition(Units.degreesToRotations(clamp(ArmAngle.AMP))));
      }
      case UNJAM -> {
        leftMotor.setControl(
            motionMagicRequest.withPosition(Units.degreesToRotations(clamp(ArmAngle.UNJAM))));
        rightMotor.setControl(
            motionMagicRequest.withPosition(Units.degreesToRotations(clamp(ArmAngle.UNJAM))));
      }
      case PASS -> {
        leftMotor.setControl(
            pidRequest.withPosition(Units.degreesToRotations(clamp(ArmAngle.PASS))));
        rightMotor.setControl(
            pidRequest.withPosition(Units.degreesToRotations(clamp(ArmAngle.PASS))));
      }
      case HOLD_FOR_INTAKE -> {
        leftMotor.setVoltage(-0.5);
        ;
        rightMotor.setVoltage(-0.5);
      }
      default -> {}
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    switch (getState()) {
      case SPEAKER_SHOT -> {
        var newAngle =
            Units.degreesToRotations(clamp(speakerDistanceToAngle.get(distanceToSpeaker)));
        leftMotor.setControl(pidRequest.withPosition(newAngle));
        rightMotor.setControl(pidRequest.withPosition(newAngle));
      }
      case FEEDING -> {
        double newAngle =
            Units.degreesToRotations(clamp(feedSpotDistanceToAngle.get(distanceToFeedSpot)));
        leftMotor.setControl(pidRequest.withPosition(newAngle));
        rightMotor.setControl(pidRequest.withPosition(newAngle));
      }
      default -> {}
    }

    DogLog.log("Arm/Left/StatorCurrent", leftMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Arm/Left/SupplyCurrent", leftMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log(
        "Arm/Left/ArmAngle", Units.rotationsToDegrees(leftMotor.getPosition().getValueAsDouble()));
    DogLog.log("Arm/Left/AppliedVoltage", leftMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Arm/Left/LowestSeenAngle", lowestSeenAngleLeft);
    DogLog.log(
        "Arm/Left/GoalAngle",
        Units.rotationsToDegrees(leftMotor.getClosedLoopReference().getValueAsDouble()));

    DogLog.log("Arm/Right/StatorCurrent", rightMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Arm/Right/SupplyCurrent", rightMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log(
        "Arm/Right/ArmAngle",
        Units.rotationsToDegrees(rightMotor.getPosition().getValueAsDouble()));
    DogLog.log("Arm/Right/AppliedVoltage", rightMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Arm/Right/LowestSeenAngle", lowestSeenAngleRight);
    DogLog.log(
        "Arm/Right/GoalAngle",
        Units.rotationsToDegrees(rightMotor.getClosedLoopReference().getValueAsDouble()));
    DogLog.log("Arm/DistanceToSpeaker", distanceToSpeaker);

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
