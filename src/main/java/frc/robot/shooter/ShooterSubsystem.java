package frc.robot.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ShooterSubsystem extends StateMachine<ShooterState> {
  private final TalonFX motor;
  private double shooterRPM;
  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0).withEnableFOC(false).withLimitReverseMotion(true);
  private double distanceToSpeaker;
  private double distanceToFeedSpot;
  private InterpolatingDoubleTreeMap speakerDistanceToRpm = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap feedSpotDistanceToRpm = new InterpolatingDoubleTreeMap();

  public void setDistanceToSpeaker(double distance) {
    distanceToSpeaker = distance;
  }

  public void setDistanceToFeedSpot(double distance) {
    distanceToFeedSpot = distance;
  }

  public ShooterSubsystem(TalonFX motor) {
    super(SubsystemPriority.SHOOTER, ShooterState.IDLE_NO_GP);
    this.motor = motor;

    // TODO: Tune lookup table
    speakerDistanceToRpm.put(123.0, 321.0);

    feedSpotDistanceToRpm.put(123.0, 321.0);
  }

  public boolean atGoal() {
    return switch (getState()) {
      case SUBWOOFER_SHOT -> MathUtil.isNear(ShooterRpms.SUBWOOFER, shooterRPM, 50);
      case IDLE_WITH_GP, IDLE_NO_GP -> true;
      case PODIUM_SHOT -> MathUtil.isNear(ShooterRpms.PODIUM, shooterRPM, 50);
      case DROP -> MathUtil.isNear(ShooterRpms.DROP, shooterRPM, 50);
      case FEEDING ->
          MathUtil.isNear(feedSpotDistanceToRpm.get(distanceToFeedSpot), shooterRPM, 50);
      case SPEAKER_SHOT ->
          MathUtil.isNear(speakerDistanceToRpm.get(distanceToSpeaker), shooterRPM, 50);
    };
  }

  public void setState(ShooterState newState) {
    setStateFromRequest(newState);
  }

  @Override
  protected void collectInputs() {
    shooterRPM = motor.getVelocity().getValueAsDouble() * 60.0;
  }

  @Override
  protected ShooterState getNextState(ShooterState currentState) {
    // The shooter has no sensor, so some other manager will need to set shooter state
    return currentState;
  }

  @Override
  protected void afterTransition(ShooterState newState) {
    switch (newState) {
      case IDLE_NO_GP -> motor.disable();
      case IDLE_WITH_GP ->
          motor.setControl(velocityRequest.withVelocity(ShooterRpms.IDLE_WITH_GP / 60.0));
      case SPEAKER_SHOT ->
          motor.setControl(
              velocityRequest.withVelocity(speakerDistanceToRpm.get(distanceToSpeaker) / 60.0));
      case SUBWOOFER_SHOT ->
          motor.setControl(velocityRequest.withVelocity(ShooterRpms.SUBWOOFER / 60.0));
      case FEEDING ->
          motor.setControl(
              velocityRequest.withVelocity(speakerDistanceToRpm.get(distanceToFeedSpot) / 60.0));
      case DROP -> motor.setControl(velocityRequest.withVelocity(ShooterRpms.DROP / 60.0));
      case PODIUM_SHOT -> motor.setControl(velocityRequest.withVelocity(ShooterRpms.PODIUM / 60.0));
    }
  }

  @Override
  public void robotPeriodic() {}
}
