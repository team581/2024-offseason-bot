package frc.robot.robot_manager;

import frc.robot.arm.ArmState;
import frc.robot.arm.ArmSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.queuer.QueuerState;
import frc.robot.queuer.QueuerSubsystem;
import frc.robot.shooter.ShooterState;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.VisionSubsystem;

public class RobotManager extends StateMachine<RobotState> {
  public final ArmSubsystem arm;
  public final ShooterSubsystem shooter;
  public final LocalizationSubsystem localization;
  public final VisionSubsystem vision;
  public final ImuSubsystem imu;
  public final IntakeSubsystem intake;
  public final QueuerSubsystem queuer;

  private final double distanceToFeedSpot = 0;
  private final double distanceToSpeaker = 0;

  public RobotManager(
      ArmSubsystem arm,
      ShooterSubsystem shooter,
      LocalizationSubsystem localization,
      VisionSubsystem vision,
      ImuSubsystem imu,
      IntakeSubsystem intake,
      QueuerSubsystem queuer) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE_NO_GP);
    this.arm = arm;
    this.shooter = shooter;
    this.localization = localization;
    this.vision = vision;
    this.imu = imu;
    this.intake = intake;
    this.queuer = queuer;
  }
  
  @Override
  protected void collectInputs() {}

  protected RobotState getNexState(RobotState currentState){
    // state transition
    switch (currentState) {
      case SPEAKER_PREPARE_TO_SCORE -> {
        if (shooter.atGoal()&&arm.atGoal()) {
          setStateFromRequest(RobotState.SPEAKER_SCORING);
        }
      }
      case AMP_PREPARE_TO_SCORE->{
        if (shooter.atGoal()&&arm.atGoal()){
          setStateFromRequest(RobotState.AMP_SCORING);
        }
      }
      case FEEDING_PREPARE_TO_SHOOT ->{
        if (shooter.atGoal()&&arm.atGoal()){
          setStateFromRequest(RobotState.FEEDING_SHOOTING);
        }
        
      }
      case PASS_PREPARE_TO_SHOOT->{
        if (shooter.atGoal()&&arm.atGoal()){
          setStateFromRequest(RobotState.PASS_SHOOTING);
        }
      }
      

    }
    return currentState;
  }
  @Override
  protected void afterTransition(RobotState newState){
        // on state change
    switch (newState) {
      case SPEAKER_PREPARE_TO_SCORE -> {
        arm.setState(ArmState.SPEAKER_SHOT);
        shooter.setState(ShooterState.SPEAKER_SHOT);
        intake.setState(IntakeState.IDLE);
        queuer.seState(QueuerState.IDLE_WITH_GP);
      }
      case AMP_PREPARE_TO_SCORE -> {
        arm.setState(ArmState.AMP);
        shooter.setState(ShooterState.AMP);
        intake.setState(IntakeState.IDLE);
        queuer.seState(QueuerState.IDLE_WITH_GP);
      }
      case FEEDING_PREPARE_TO_SHOOT -> {
        arm.setState(ArmState.FEEDING);
        shooter.setState(ShooterState.FEEDING);
        intake.setState(IntakeState.IDLE);
        queuer.seState(QueuerState.IDLE_WITH_GP);
      }
      case PASS_PREPARE_TO_SHOOT -> {
        arm.setState(ArmState.PASS);
        shooter.setState(ShooterState.PASS);
        intake.setState(IntakeState.IDLE);
        queuer.seState(QueuerState.IDLE_WITH_GP);
      }
      
    }
    }
  

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    

    // continous sate action
    switch (getState()) {
      case SPEAKER_PREPARE_TO_SCORE -> {
        shooter.setDistanceToFeedSpot(distanceToFeedSpot);
        shooter.setDistanceToSpeaker(distanceToSpeaker);
      }
      case FEEDING_PREPARE_TO_SHOOT -> {
        shooter.setDistanceToFeedSpot(distanceToFeedSpot);
        shooter.setDistanceToSpeaker(distanceToSpeaker);
      }
      case AMP_PREPARE_TO_SCORE,
       PASS_PREPARE_TO_SHOOT->{}
    }
  }

}