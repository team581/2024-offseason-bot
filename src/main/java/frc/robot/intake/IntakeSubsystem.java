package frc.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import dev.doglog.DogLog;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class IntakeSubsystem extends StateMachine<IntakeState> {
  private final TalonFX mainMotor;
  private final CANSparkMax centeringMotor;

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

  public double getIntakeRotations() {
    return mainMotor.getRotorPosition().getValueAsDouble();
  }

  @Override
  protected IntakeState getNextState(IntakeState currentState) {
    return currentState;
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      case IDLE -> {
        mainMotor.disable();
        centeringMotor.disable();
      }

      case INTAKING -> {
        mainMotor.setVoltage(12);
        centeringMotor.setVoltage(8);
      }

      case OUTTAKING -> {
        mainMotor.setVoltage(-6);
        centeringMotor.setVoltage(-10);
      }
      case INTAKING_BACK -> {
        mainMotor.setVoltage(-0.5);
      }
      case INTAKING_FORWARD_PUSH -> {
        mainMotor.setVoltage(0.5);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Intake/StatorCurrent", mainMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/SupplyCurrent", mainMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Intake/AppliedVoltage", mainMotor.getMotorVoltage().getValueAsDouble());
  }
}
