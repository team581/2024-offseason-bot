package frc.robot.reefscape_intake;

import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class PrototypeIntakeSubsystem extends StateMachine<PrototypeState> {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  public PrototypeIntakeSubsystem(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.PROTOTYPE, PrototypeState.IDLE);

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    leftMotor.getConfigurator().apply(RobotConfig.get().prototype().leftMotorConfig());
    rightMotor.getConfigurator().apply(RobotConfig.get().prototype().rightMotorConfig());
  }

  public void setState(PrototypeState newState) {
    setStateFromRequest(newState);
  }

  @Override
  protected PrototypeState getNextState(PrototypeState currentState) {
    return currentState;
  }

  @Override
  protected void afterTransition(PrototypeState newState) {
    switch (newState) {
      case IDLE -> {
        leftMotor.disable();
        rightMotor.disable();
      }

      case INTAKE -> {
        leftMotor.setVoltage(-6);
        rightMotor.setVoltage(6);
      }

      case OUTTAKE -> {
        leftMotor.setVoltage(6);
        rightMotor.setVoltage(-6);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Prototype/State", getState());
  }
}
