package frc.robot.util.state_machines;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class StateHandler<S extends Enum<S>> {
  private final Map<S, StateHandler<S>> stateHandlers;

  private Runnable onEnterAction = () -> {};
  private Supplier<S> withTransitionsAction = () -> null;
  private Consumer<S> onExitAction = (nextState) -> {};
  private Runnable periodicAction = () -> {};

  // TODO: Consider adding in a state-specific collectInputs() method

  StateHandler(Map<S, StateHandler<S>> stateHandlers) {
    this.stateHandlers = stateHandlers;
  }

  public StateHandler<S> onEnter(Runnable onEnterAction) {
    this.onEnterAction = onEnterAction;
    return this;
  }

  public StateHandler<S> onEnter(S state) {
    this.onEnterAction = stateHandlers.get(state).onEnterAction;
    return this;
  }

  public StateHandler<S> withTransitions(Supplier<S> withTransitionsAction) {
    this.withTransitionsAction = withTransitionsAction;
    return this;
  }

  public StateHandler<S> withTransitions(S state) {
    this.withTransitionsAction = () -> state;
    return this;
  }

  public StateHandler<S> onExit(Consumer<S> onExitAction) {
    this.onExitAction = onExitAction;
    return this;
  }

  public StateHandler<S> onExit(S state) {
    this.onExitAction = stateHandlers.get(state).onExitAction;
    return this;
  }

  public StateHandler<S> withPeriodic(Runnable periodicAction) {
    this.periodicAction = periodicAction;
    return this;
  }

  public StateHandler<S> withPeriodic(S state) {
    this.periodicAction = stateHandlers.get(state).periodicAction;
    return this;
  }

  void doOnEnter() {
    onEnterAction.run();
  }

  S doTransitions() {
    return withTransitionsAction.get();
  }

  void doOnExit(S nextState) {
    onExitAction.accept(nextState);
  }

  void doPeriodic() {
    periodicAction.run();
  }
}
