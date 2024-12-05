package frc.robot.autos.trailblazer.trackers.pure_pursuit;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class PurePursuitUtilsTest {
  @Test
  void addTwoNumbers() {
    var a = 1.0;
    var b = 3.0;

    var result = PurePursuitUtils.add(a, b);
    var expected = 4.0;

    Assertions.assertEquals(expected, result);
  }
}
