package frc.robot.shooter;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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

    createHandler(ShooterState.IDLE_STOPPED)
        .onEnter(
            () -> {
              topMotor.disable();
              bottomMotor.disable();
            });
    createHandler(ShooterState.IDLE_WARMUP)
        .onEnter(
            () -> {
              topMotor.setControl(velocityRequest.withVelocity(ShooterRpms.IDLE_WARMUP / 60.0));
              bottomMotor.setControl(velocityRequest.withVelocity(ShooterRpms.IDLE_WARMUP / 60.0));
            });
    createHandler(ShooterState.SUBWOOFER_SHOT)
        .onEnter(
            () -> {
              topMotor.setControl(velocityRequest.withVelocity(ShooterRpms.SUBWOOFER / 60.0));
              bottomMotor.setControl(velocityRequest.withVelocity(ShooterRpms.SUBWOOFER / 60.0));
            });
    createHandler(ShooterState.DROP)
        .onEnter(
            () -> {
              topMotor.setControl(velocityRequest.withVelocity(ShooterRpms.DROP / 60.0));
              bottomMotor.setControl(velocityRequest.withVelocity(ShooterRpms.DROP / 60.0));
            });
    createHandler(ShooterState.PODIUM_SHOT)
        .onEnter(
            () -> {
              topMotor.setControl(velocityRequest.withVelocity(ShooterRpms.PODIUM / 60.0));
              bottomMotor.setControl(velocityRequest.withVelocity(ShooterRpms.PODIUM / 60.0));
            });
    createHandler(ShooterState.PASS)
        .onEnter(
            () -> {
              topMotor.setControl(velocityRequest.withVelocity(ShooterRpms.PASS / 60.0));
              bottomMotor.setControl(velocityRequest.withVelocity(ShooterRpms.PASS / 60.0));
            });
    createHandler(ShooterState.SPEAKER_SHOT)
        .withPeriodic(
            () -> {
              var goalRpm = speakerDistanceToRpm.get(distanceToSpeaker);
              topMotor.setControl(velocityRequest.withVelocity(goalRpm / 60.0));
              bottomMotor.setControl(velocityRequest.withVelocity(goalRpm / 60.0));
            });
    createHandler(ShooterState.FEEDING)
        .withPeriodic(
            () -> {
              var goalRpm = speakerDistanceToRpm.get(distanceToFeedSpot);
              topMotor.setControl(velocityRequest.withVelocity(goalRpm / 60.0));
              bottomMotor.setControl(velocityRequest.withVelocity(goalRpm / 60.0));
            });
  }

  public boolean atGoal() {
    return switch (getState()) {
      case SUBWOOFER_SHOT ->
          MathUtil.isNear(ShooterRpms.SUBWOOFER, topMotorRpm, 150)
              && MathUtil.isNear(ShooterRpms.SUBWOOFER, bottomMotorRpm, 50);
      case IDLE_WARMUP, IDLE_STOPPED -> true;
      case PODIUM_SHOT ->
          MathUtil.isNear(ShooterRpms.PODIUM, topMotorRpm, 150)
              && MathUtil.isNear(ShooterRpms.PODIUM, bottomMotorRpm, 150);
      case DROP ->
          MathUtil.isNear(ShooterRpms.DROP, topMotorRpm, 150)
              && MathUtil.isNear(ShooterRpms.DROP, bottomMotorRpm, 150);
      case FEEDING ->
          MathUtil.isNear(feedSpotDistanceToRpm.get(distanceToFeedSpot), bottomMotorRpm, 150)
              && MathUtil.isNear(feedSpotDistanceToRpm.get(distanceToFeedSpot), topMotorRpm, 150);
      case SPEAKER_SHOT ->
          MathUtil.isNear(speakerDistanceToRpm.get(distanceToSpeaker), bottomMotorRpm, 150)
              && MathUtil.isNear(speakerDistanceToRpm.get(distanceToSpeaker), topMotorRpm, 150);
      case PASS ->
          MathUtil.isNear(ShooterRpms.PASS, bottomMotorRpm, 150)
              && MathUtil.isNear(ShooterRpms.PASS, topMotorRpm, 150);
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
  public void robotPeriodic() {
    super.robotPeriodic();

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
