package frc.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class IntakeSubsystem extends StateMachine<IntakeState> {
    private final TalonFX motor;
    private final DigitalInput sensor;
    private boolean sensorHasNote = false;
    private boolean debouncedSensorHasNote = false;
    private final Debouncer debouncer = new Debouncer(3.0*0.02);

    public IntakeSubsystem (TalonFX motor,DigitalInput sensor){
        super(SubsystemPriority.INTAKE, IntakeState.IDLE);

        this.sensor = sensor;
        this.motor = motor;
    }
    public void setState (IntakeState newState){
        setStateFromRequest(newState);
    }
    
    @Override
    protected void collectInputs(){
        sensorHasNote=sensor.get();
        debouncedSensorHasNote=debouncer.calculate(sensorHasNote);
    }
    @Override
    protected IntakeState getNextState(IntakeState currentState){
        return currentState;
    }
    public boolean hasNote(){
        return debouncedSensorHasNote;
    }
    @Override
    protected void afterTransition(IntakeState newState){
        switch (newState){
            case IDLE->motor.disable();
            case INTAKING->motor.setVoltage(0);//around 10
            case OUTTAKING->motor.setVoltage(0);//around -6
        }
    }
    @Override
    public void robotPeriodic(){
        super.robotPeriodic();
        DogLog.log("Intake/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
        DogLog.log("Intake/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
        DogLog.log("Intake/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
        DogLog.log("Intake/RawSensor",sensorHasNote);
        DogLog.log("Intake/DebouncedSensor",hasNote());
        
    }
}
