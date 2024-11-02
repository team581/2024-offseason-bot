package frc.robot.util.state_machines;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystemManager;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.EnumMap;
import java.util.Set;

/** A state machine backed by {@link LifecycleSubsystem}. */
public abstract class StateMachine<S extends Enum<S>> extends LifecycleSubsystem {
  private S previousState = null;
  private S currentState;
  private double lastTransitionTimestamp = Timer.getFPGATimestamp();
  private final EnumMap<S, StateHandler<S>> stateHandlers;
  private StateHandler<S> currentStateHandler;

  /**
   * Creates a new state machine.
   *
   * @param priority The subsystem priority of this subsystem in {@link LifecycleSubsystemManager}.
   * @param initialState The initial/default state of the state machine.
   */
  protected StateMachine(SubsystemPriority priority, S initialState) {
    super(priority);
    currentState = initialState;
    stateHandlers = new EnumMap<>(initialState.getDeclaringClass());

    // Ensures that all states have a default handler
    for (S state : initialState.getDeclaringClass().getEnumConstants()) {
      stateHandlers.put(state, new StateHandler<>(stateHandlers));
    }

    currentStateHandler = stateHandlers.get(initialState);
  }

  /** Processes collecting inputs, state transitions, and state actions. */
  @Override
  public void robotPeriodic() {
    collectInputs();

    // A transition occurred last time, so we handle the exit and enter actions
    if (previousState != currentState) {
      lastTransitionTimestamp = Timer.getFPGATimestamp();
      currentStateHandler.doOnEnter();

      if (previousState != null) {
        var previousStateHandler = stateHandlers.get(previousState);
        previousStateHandler.doOnExit(currentState);
      }
    }

    currentStateHandler.doPeriodic();

    var nextStateOrNull = currentStateHandler.doTransitions();

    // A transition should occur next loop
    if (nextStateOrNull != null) {
      previousState = currentState;
      currentState = nextStateOrNull;
      currentStateHandler = stateHandlers.get(currentState);
    }
  }

  public StateHandler<S> createHandler(S state) {
    return stateHandlers.get(state);
  }

  /**
   * Gets the current state.
   *
   * @return The current state.
   */
  public S getState() {
    return currentState;
  }

  /**
   * Creates a command that waits until this state machine is in the given state.
   *
   * @param goalState The state to wait for.
   * @return A command that waits until the state is equal to the goal state.
   */
  public Command waitForState(S goalState) {
    return Commands.waitUntil(() -> this.currentState == goalState);
  }

  /**
   * Creates a command that waits until this state machine is in any of the given states.
   *
   * @param goalStates A set of the states to wait for.
   * @return A command that waits until the state is equal to any of the goal states.
   */
  public Command waitForStates(Set<S> goalStates) {
    return Commands.waitUntil(() -> goalStates.contains(this.currentState));
  }

  /**
   * Called each loop before processing transitions. Used for retrieving sensor values, etc.
   *
   * <p>Default behavior is to do nothing.
   */
  protected void collectInputs() {}

  /**
   * Used to change to a new state when a request is made. Will also trigger all logic that should
   * happen when a state transition occurs.
   *
   * @param requestedState The new state to transition to.
   */
  protected void setStateFromRequest(S requestedState) {
    currentState = requestedState;
  }

  /**
   * Checks if the current state has been in for longer than the given duration. Used for having
   * timeout logic in state transitions.
   *
   * @param duration The timeout duration (in seconds) to use.
   * @return Whether the current state has been active for longer than the given duration.
   */
  protected boolean timeout(double duration) {
    var currentStateDuration = Timer.getFPGATimestamp() - lastTransitionTimestamp;

    return currentStateDuration > duration;
  }
}
