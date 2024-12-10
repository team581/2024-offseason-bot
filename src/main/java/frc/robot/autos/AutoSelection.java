package frc.robot.autos;

import java.util.function.BiFunction;

import frc.robot.autos.amp.Op345Auto;
import frc.robot.autos.amp.TestAuto;
import frc.robot.autos.trailblazer.Trailblazer;
import frc.robot.robot_manager.RobotManager;

public enum AutoSelection {
  DO_NOTHING(DoNothingAuto::new),

  OP(Op345Auto::new),
  PURE_PURSUIT_TEST_AUTO(TestAuto::new);

  public final BiFunction<RobotManager, Trailblazer, BaseAuto> auto;

  private AutoSelection(BiFunction<RobotManager, Trailblazer, BaseAuto> auto) {
    this.auto = auto;
  }
}
