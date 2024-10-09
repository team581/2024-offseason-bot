package frc.robot.shooter;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ShooterSubsystem extends StateMachine<ShooterState> {
  private final TalonFX topMotor;
  private final TalonFX bottomMotor;
  private double topMotorRpm;
  private double bottomMotorRpm;
  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0).withLimitReverseMotion(true);
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

  public ShooterSubsystem(TalonFX topMotor, TalonFX bottomMotor) {
    super(SubsystemPriority.SHOOTER, ShooterState.IDLE_STOPPED);
    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;
    topMotor.getConfigurator().apply(RobotConfig.get().shooter().topMotorConfig());
    bottomMotor.getConfigurator().apply(RobotConfig.get().shooter().bottomMotorConfig());

    RobotConfig.get().shooter().feedSpotDistanceToRpm().accept(feedSpotDistanceToRpm);
    RobotConfig.get().shooter().speakerDistanceToRpm().accept(speakerDistanceToRpm);
  }

  public boolean atGoal() {
    return switch (getState()) {
      case SUBWOOFER_SHOT ->
          MathUtil.isNear(ShooterRpms.SUBWOOFER, topMotorRpm, 50)
              && MathUtil.isNear(ShooterRpms.SUBWOOFER, bottomMotorRpm, 50);
      case IDLE_WARMUP, IDLE_STOPPED -> true;
      case PODIUM_SHOT ->
          MathUtil.isNear(ShooterRpms.PODIUM, topMotorRpm, 50)
              && MathUtil.isNear(ShooterRpms.PODIUM, bottomMotorRpm, 50);
      case DROP ->
          MathUtil.isNear(ShooterRpms.DROP, topMotorRpm, 50)
              && MathUtil.isNear(ShooterRpms.DROP, bottomMotorRpm, 50);
      case FEEDING ->
          MathUtil.isNear(feedSpotDistanceToRpm.get(distanceToFeedSpot), bottomMotorRpm, 50)
              && MathUtil.isNear(feedSpotDistanceToRpm.get(distanceToFeedSpot), topMotorRpm, 50);
      case SPEAKER_SHOT ->
          MathUtil.isNear(speakerDistanceToRpm.get(distanceToSpeaker), bottomMotorRpm, 50)
              && MathUtil.isNear(speakerDistanceToRpm.get(distanceToSpeaker), topMotorRpm, 50);
      case AMP ->
          MathUtil.isNear(ShooterRpms.AMP, bottomMotorRpm, 50)
              && MathUtil.isNear(ShooterRpms.AMP, topMotorRpm, 50);
      case PASS ->
          MathUtil.isNear(ShooterRpms.PASS, bottomMotorRpm, 50)
              && MathUtil.isNear(ShooterRpms.PASS, topMotorRpm, 50);
    };
  }

  public void setState(ShooterState newState) {
    setStateFromRequest(newState);
  }

  @Override
  protected void collectInputs() {
    topMotorRpm = topMotor.getVelocity().getValueAsDouble() * 60.0;
    bottomMotorRpm = bottomMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  @Override
  protected void afterTransition(ShooterState newState) {
    switch (newState) {
      case IDLE_STOPPED -> topMotor.disable();
      case IDLE_WARMUP -> {
        topMotor.setControl(velocityRequest.withVelocity(ShooterRpms.IDLE_WARMUP / 60.0));
        bottomMotor.setControl(velocityRequest.withVelocity(ShooterRpms.IDLE_WARMUP / 60.0));
      }

      case SUBWOOFER_SHOT -> {
        topMotor.setControl(velocityRequest.withVelocity(ShooterRpms.SUBWOOFER / 60.0));
        bottomMotor.setControl(velocityRequest.withVelocity(ShooterRpms.SUBWOOFER / 60.0));
      }

      case DROP -> {
        topMotor.setControl(velocityRequest.withVelocity(ShooterRpms.DROP / 60.0));
        bottomMotor.setControl(velocityRequest.withVelocity(ShooterRpms.DROP / 60.0));
      }
      case PODIUM_SHOT -> {
        topMotor.setControl(velocityRequest.withVelocity(ShooterRpms.PODIUM / 60.0));
        bottomMotor.setControl(velocityRequest.withVelocity(ShooterRpms.PODIUM / 60.0));
      }
      case AMP -> {
        topMotor.setControl(velocityRequest.withVelocity(ShooterRpms.AMP / 60.0));
        bottomMotor.setControl(velocityRequest.withVelocity(ShooterRpms.AMP / 60.0));
      }
      case PASS -> {
        topMotor.setControl(velocityRequest.withVelocity(ShooterRpms.PASS / 60.0));
        bottomMotor.setControl(velocityRequest.withVelocity(ShooterRpms.PASS / 60.0));
      }
      default -> {}
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    switch (getState()) {
      case SPEAKER_SHOT -> {
        var goalRpm = speakerDistanceToRpm.get(distanceToSpeaker);
        topMotor.setControl(velocityRequest.withVelocity(goalRpm / 60.0));
        bottomMotor.setControl(velocityRequest.withVelocity(goalRpm / 60.0));
      }
      case FEEDING -> {
        var goalRpm = speakerDistanceToRpm.get(distanceToFeedSpot);
        topMotor.setControl(velocityRequest.withVelocity(goalRpm / 60.0));
        bottomMotor.setControl(velocityRequest.withVelocity(goalRpm / 60.0));
      }
      default -> {}
    }

    DogLog.log("Shooter/Top/StatorCurrent", topMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Shooter/Top/SupplyCurrent", topMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Shooter/Top/RPM", topMotor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Shooter/Top/AppliedVoltage", topMotor.getMotorVoltage().getValueAsDouble());

    DogLog.log("Shooter/Bottom/StatorCurrent", bottomMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Shooter/Bottom/SupplyCurrent", bottomMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Shooter/Bottom/RPM", bottomMotor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Shooter/Bottom/AppliedVoltage", bottomMotor.getMotorVoltage().getValueAsDouble());

    var goalRpm = topMotor.getClosedLoopReference().getValueAsDouble() * 60.0;


    DogLog.log("Shooter/GoalRPM", goalRpm);
  }
}
