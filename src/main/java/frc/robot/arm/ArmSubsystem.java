package frc.robot.arm;

import javax.swing.text.Position;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.queuer.QueuerState;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ArmSubsystem extends StateMachine<ArmState> {
  private final TalonFX motor;
  private double armAngle;
  private double angleToSpeaker;
  private double angleToFeedSpot;
  private InterpolatingDoubleTreeMap speakerDistanceToAngle = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap feedSpotDistanceToAngle = new InterpolatingDoubleTreeMap();
  private final PositionVoltage positionRequest =
      new PositionVoltage(0).withEnableFOC(false).withLimitReverseMotion(false);


  public void setAngleToSpeaker(double angle){
    angleToSpeaker = angle;
  }
  public void setAngleToFeedSpot(double angle){
    angleToFeedSpot = angle;
  }


  public ArmSubsystem(TalonFX motor) {
    super(SubsystemPriority.ARM, ArmState.IDLE);
    this.motor=motor;
    speakerDistanceToAngle.put(123.0,321.0);
    feedSpotDistanceToAngle.put(123.0,321.0);

  }
  public void setState(ArmState newState){
    setStateFromRequest(newState);
  }
  public boolean atGoal(){
    return switch (getState()){
      case IDLE->MathUtil.isNear.(ArmAngle.IDLE.getDegrees(),armAngle,1);
      case SPEAKER_SHOT->MathUtil.isNear(speakerDistanceToAngle.get(angleToSpeaker),armAngle,1);
      case DROP->MathUtil.isNear(ArmAngle.DROP,armAngle,1);
      case SUBWOOFER_SHOT->MathUtil.isNear(ArmAngle.SUBWOOFER,armAngle,1);
      case FEEDING->MathUtil.isNear(feedSpotDistanceToAngle.get(angleToFeedSpot),armAngle,1);
      case PODIUM_SHOT->MathUtil.isNear(ArmAngle.PODIUM,armAngle,1);
      case CLIMBING->MathUtil.isNear(ArmAngle.CLIMBING,armAngle,1);

    };
  }

  @Override
  protected void collectInputs(){
    armAngle = motor.getPosition().getValueAsDouble();//is get position get angle??
  }
  @Override
  protected void afterTransition(ArmState newState){
    switch(newState){
      case IDLE -> motor.disable();
      case CLIMBING->motor.setControl(positionRequest.withPosition(ArmAngle.CLIMBING));
      case DROP->motor.setControl(positionRequest.withPosition(ArmAngle.DROP));
    }
  }
  @Override
  public void robotPeriodic(){
    super.robotPeriodic();
  }

}
