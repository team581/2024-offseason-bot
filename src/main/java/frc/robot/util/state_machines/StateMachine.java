// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.state_machines;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystemManager;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.Set;

/** A state machine backed by {@link LifecycleSubsystem}. */
public abstract class StateMachine<S extends Enum<S>> extends LifecycleSubsystem {
  private S state;

  /**
   * Creates a new state machine.
   *
   * @param priority The subsystem priority of this subsystem in {@link LifecycleSubsystemManager}.
   * @param initialState The initial/default state of the state machine.
   */
  protected StateMachine(SubsystemPriority priority, S initialState) {
    super(priority);
    state = initialState;
  }

  /** Processes collecting inputs, state transitions, and state actions. */
  @Override
  public void robotPeriodic() {
    DogLog.log(subsystemName + "/State", state);

    collectInputs();

    var stateBeforeTransitions = state;
    state = getNextState(state);

    if (state != stateBeforeTransitions) {
      DogLog.log(subsystemName + "/StateAfterTransition", state);
      afterTransition(state);
    } else {
      DogLog.log(subsystemName + "/StateAfterTransition", "(no change)");
    }
  }

  /**
   * Called each loop before processing transitions. Used for retrieving sensor values, etc.
   *
   * <p>Default behavior is to do nothing.
   */
  protected void collectInputs() {}

  /**
   * Process transitions from one state to another.
   *
   * <p>Default behavior is to stay in the current state indefinitely.
   *
   * @param currentState The current state.
   * @return The new state after processing transitions.
   */
  protected S getNextState(S currentState) {
    return currentState;
  }

  /**
   * Runs once after entering a new state. This is where you should run state actions.
   *
   * @param newState The newly entered state.
   */
  protected void afterTransition(S newState) {}

  /**
   * Used to change to a new state when a request is made.
   *
   * @param requestedState The new state to transition to.
   */
  protected void setStateFromRequest(S requestedState) {
    this.state = requestedState;
  }

  /**
   * Gets the current state.
   *
   * @return The current state.
   */
  public S getState() {
    return state;
  }

  /**
   * Creates a command that waits until this state machine is in the given state.
   *
   * @param goalState The state to wait for.
   * @return A command that waits until the state is equal to the goal state.
   */
  public Command waitForState(S goalState) {
    return Commands.waitUntil(() -> this.state == goalState);
  }

  /**
   * Creates a command that waits until this state machine is in any of the given states.
   *
   * @param goalStates A set of the states to wait for.
   * @return A command that waits until the state is equal to any of the goal states.
   */
  public Command waitForStates(Set<S> goalStates) {
    return Commands.waitUntil(() -> goalStates.contains(this.state));
  }
}
