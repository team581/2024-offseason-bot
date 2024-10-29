package frc.robot.lights;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class LightsSubsystem extends LifecycleSubsystem {
  private BlinkPattern blinkPattern;
  private Color color;

  public LightsSubsystem() {
    super(SubsystemPriority.LIGHTS);
  }

  @Override
  public void robotPeriodic() {
    // TODO: Check if we are disabled, and do something different there

    // Continuous state actions
    switch (robotManager.getState()) {
      case INTAKING -> {
        blinkPattern = BlinkPattern.SOLID;
        color = Color.kRed;
      }
      case INTAKING_BACK, INTAKING_FORWARD_PUSH -> {
        blinkPattern = BlinkPattern.SOLID;
        color = Color.kYellow;
      }
    }

    // TODO: Do something with the blink & color (set CANdle)
  }
}
