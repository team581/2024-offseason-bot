package frc.robot.queuer;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class QueuerSubsystem extends StateMachine<QueuerState> {
  private final TalonFX motor;
  private final DigitalInput sensor;
  private boolean sensorHasNote = false;
  private boolean debouncedSensorHasNote = false;
  private double motorPosition;
  private double goalPosition;
  private final Debouncer debouncer = RobotConfig.get().queuer().debouncer();

  private final PositionVoltage pidRequest = new PositionVoltage(0).withEnableFOC(false);

  public QueuerSubsystem(TalonFX motor, DigitalInput sensor) {
    super(SubsystemPriority.QUEUER, QueuerState.IDLE);

    this.sensor = sensor;
    this.motor = motor;

    motor.getConfigurator().apply(RobotConfig.get().queuer().motorConfig());
  }

  public void setState(QueuerState newState) {
    setStateFromRequest(newState);
  }

  public boolean atGoal() {
    return switch (getState()) {
      case INTAKING_FORWARD_PUSH -> MathUtil.isNear(goalPosition, motorPosition, 0.4);
      default -> true;
    };
  }

  @Override
  protected void collectInputs() {
    sensorHasNote = sensor.get();
    debouncedSensorHasNote = debouncer.calculate(sensorHasNote);
    motorPosition = motor.getRotorPosition().getValueAsDouble();
  }

  @Override
  protected QueuerState getNextState(QueuerState currentState) {
    // State transitions are done by robot manager, not here
    return currentState;
  }

  public boolean hasNote() {
    return debouncedSensorHasNote;
  }

  @Override
  protected void afterTransition(QueuerState newState) {
    switch (newState) {
      case IDLE -> motor.disable();
      case SHOOTING -> motor.setVoltage(12);
      case INTAKING -> motor.setVoltage(3.5);
      case OUTTAKING -> motor.setVoltage(-4);
      case AMPING -> motor.setVoltage(-10);
      case INTAKING_BACK -> motor.setVoltage(-1);
      case INTAKING_FORWARD_PUSH -> {
        goalPosition = motorPosition + 2;
        motor.setControl(pidRequest.withPosition(goalPosition));
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("Queuer/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Queuer/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Queuer/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Queuer/RawSensor", sensorHasNote);
    DogLog.log("Queuer/DebouncedSensor", hasNote());
    DogLog.log("Queuer/atGoal", atGoal());
    DogLog.log("Queuer/goalPosition", goalPosition);
    DogLog.log("Queuer/motorPosition", motorPosition);
  }
}
