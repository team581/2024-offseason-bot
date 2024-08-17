package frc.robot.queuer;

import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;


public class QueuerSubsystem extends StateMachine<QueuerState> {
  private final TalonFX motor;
  private final DigitalInput sensor;
  private boolean sensorHasNote = false;
  private boolean debouncedSensorHasNote = false;
  private final Debouncer debouncer= new Debouncer(3 * 0.02);

  public QueuerSubsystem(TalonFX motor, DigitalInput sensor) {
    super(SubsystemPriority.QUEUER, QueuerState.IDLE_NO_GP);

    this.sensor = sensor;
    this.motor = motor;
  }
  public void seState(QueuerState newState){
    setStateFromRequest(newState);
  }
  @Override
  protected void collectInputs(){
    sensorHasNote=sensor.get(); 
    debouncedSensorHasNote = debouncer.calculate(sensorHasNote);
  }
  
  @Override
  protected QueuerState getNextState(QueuerState currentState){
    // State transitions are done by robot manager, not here
    return currentState;
  }

  public boolean hasNote() {
    return debouncedSensorHasNote;
  }


  @Override
  protected void afterTransition(QueuerState newState){
    switch(newState){
      case IDLE_NO_GP->motor.disable();
      case IDLE_WITH_GP->motor.disable();
      case SHOOTING->motor.setVoltage(0);//probably like 12
      case INTAKING->motor.setVoltage(0);//probably like 4
      case OUTTAKING->motor.setVoltage(0);//proabably like -4
    }
  }
  @Override
  public void robotPeriodic(){
    DogLog.log("Queuer/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Queuer/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Queuer/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
  }
}
