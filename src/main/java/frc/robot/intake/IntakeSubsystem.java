package frc.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class IntakeSubsystem extends StateMachine<IntakeState> {
  private final TalonFX mainMotor;
  private final CANSparkMax centeringMotor;
  private boolean sensorHasNote = false;
  private boolean debouncedSensorHasNote = false;
  private final Debouncer debouncer = RobotConfig.get().intake().debouncer();

  public IntakeSubsystem(TalonFX mainMotor, CANSparkMax centeringMotor) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE);

    this.mainMotor = mainMotor;
    this.centeringMotor = centeringMotor;
    mainMotor.getConfigurator().apply(RobotConfig.get().intake().mainMotorConfig());

    RobotConfig.get().intake().centeringMotorConfig().accept(centeringMotor);
  }

  public void setState(IntakeState newState) {
    setStateFromRequest(newState);
  }

  @Override
  protected void collectInputs() {
    debouncedSensorHasNote = debouncer.calculate(sensorHasNote);
  }

  @Override
  protected IntakeState getNextState(IntakeState currentState) {
    return currentState;
  }

  public boolean hasNote() {
    return debouncedSensorHasNote;
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      case IDLE:
        mainMotor.disable();
        centeringMotor.disable();
        break;
      case INTAKING:
        mainMotor.setVoltage(0); // around 10
        centeringMotor.setVoltage(0);
      case OUTTAKING:
        mainMotor.setVoltage(0); // around -6
        centeringMotor.setVoltage(0);
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Intake/StatorCurrent", mainMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/SupplyCurrent", mainMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Intake/AppliedVoltage", mainMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Intake/RawSensor", sensorHasNote);
    DogLog.log("Intake/DebouncedSensor", hasNote());
  }
}
