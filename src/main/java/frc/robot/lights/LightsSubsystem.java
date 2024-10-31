package frc.robot.lights;

import com.ctre.phoenix.led.CANdle;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.LimelightHelpers;

public class LightsSubsystem extends LifecycleSubsystem {
  private BlinkPattern blinkPattern;
  private Color color;
  private final CANdle candle;

  private final Timer blinkTimer = new Timer();
  private final Timer lightsOnExitTimer = new Timer();

  private final RobotManager robotManager;
  private static final double FAST_BLINK_DURATION = 0.08;
  private static final double SLOW_BLINK_DURATION = 0.25;

  private RobotState previousState = RobotState.IDLE_NO_GP;


  public LightsSubsystem(RobotManager robotManager, CANdle candle) {
    super(SubsystemPriority.LIGHTS);
    this.robotManager = robotManager;
    this.candle = candle;
    blinkTimer.start();
  }

  @Override
  public void robotPeriodic() {
    // TODO: Check if we are disabled, and do something different there

    // Continuous state actions
    switch (robotManager.getState()) {
      case INTAKING, INTAKE_ASSIST -> {
        blinkPattern = BlinkPattern.SOLID;
        color = Color.kRed;
      }
      case INTAKING_BACK, INTAKING_FORWARD_PUSH -> {
        blinkPattern = BlinkPattern.SOLID;
        color = Color.kYellow;
      }
      case IDLE_WITH_GP -> {
        blinkPattern = BlinkPattern.SOLID;
        color = Color.kWhite;
      }

      case AMP_WAITING, PODIUM_WAITING, FEEDING_WAITING, SUBWOOFER_WAITING, SPEAKER_WAITING -> {
        blinkPattern = BlinkPattern.SOLID;
        color = Color.kRed;
      }
      case AMP_PREPARE_TO_SCORE,
          PASS_PREPARE_TO_SHOOT,
          PODIUM_PREPARE_TO_SCORE,
          FEEDING_PREPARE_TO_SHOOT,
          SPEAKER_PREPARE_TO_SCORE,
          SUBWOOFER_PREPARE_TO_SCORE,
          OUTTAKING -> {
        blinkPattern = BlinkPattern.SOLID;
        color = Color.kYellow;
      }
      case AMP_SCORING,
          PODIUM_SCORING,
          PASS_SHOOTING,
          SPEAKER_SCORING,
          SUBWOOFER_SCORING,
          FEEDING_SHOOTING -> {
        blinkPattern = BlinkPattern.SOLID;
        color = Color.kGreen;
      }

      case CLIMBING_1_LINEUP -> {
        blinkPattern = BlinkPattern.SOLID;
        color = Color.kRed;
      }
      case CLIMBING_2_HANGING -> {
        blinkPattern = BlinkPattern.SOLID;
        color = Color.kGreen;
      }
      case UNJAM -> {
        blinkPattern = BlinkPattern.BLINK_FAST;
        color = Color.kRed;
      }
      case IDLE_NO_GP -> {
        blinkPattern = BlinkPattern.SOLID;
        color = Color.kBlack;
      }
    }
    DogLog.log("Lights/Color", color.toString());
    DogLog.log("Lights/Pattern", blinkPattern);

    Color8Bit color8Bit = new Color8Bit(color);

    if (color == Color.kWhite) {
      LimelightHelpers.setLEDMode_ForceBlink("limelight-note");
    } else {
      LimelightHelpers.setLEDMode_ForceOff("limelight-note");
    }

    if (blinkPattern == BlinkPattern.SOLID) {
      candle.setLEDs(color8Bit.red, color8Bit.green, color8Bit.blue);
    } else {
      double time = blinkTimer.get();
      double onDuration = 0;
      double offDuration = 0;

      if (blinkPattern == BlinkPattern.BLINK_FAST) {
        onDuration = FAST_BLINK_DURATION;
        offDuration = FAST_BLINK_DURATION * 2;
      } else if (blinkPattern == BlinkPattern.BLINK_SLOW) {
        onDuration = SLOW_BLINK_DURATION;
        offDuration = SLOW_BLINK_DURATION * 2;
      }

      if (time >= offDuration) {
        blinkTimer.reset();
        candle.setLEDs(0, 0, 0);
      } else if (time >= onDuration) {
        candle.setLEDs(color8Bit.red, color8Bit.green, color8Bit.blue);
      }
    }
    previousState=robotManager.getState();
  }
}
