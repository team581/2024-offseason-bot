package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.arm.ArmSubsystem;
import frc.robot.autos.Autos;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.generated.BuildConstants;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.queuer.QueuerSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.Stopwatch;
import frc.robot.util.scheduling.LifecycleSubsystemManager;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private final FmsSubsystem fms = new FmsSubsystem();
  private final Hardware hardware = new Hardware();
  private final QueuerSubsystem queuer =
      new QueuerSubsystem(hardware.queuer, hardware.queuerSensor);
  private final ShooterSubsystem shooter =
      new ShooterSubsystem(hardware.shooterTop, hardware.shooterBottom);
  private final ArmSubsystem arm = new ArmSubsystem(hardware.armLeft, hardware.armRight);
  private final IntakeSubsystem intake =
      new IntakeSubsystem(hardware.intakeMain, hardware.intakeCenteringMotor);
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ImuSubsystem imu = new ImuSubsystem(swerve.drivetrainPigeon);
  private final Autos autos = new Autos();
  private final RobotCommands robotCommands = new RobotCommands(null);

  public Robot() {
    System.out.println("roboRIO serial number: " + RobotConfig.SERIAL_NUMBER);

    DogLog.setOptions(
        new DogLogOptions().withCaptureNt(false).withNtPublish(RobotConfig.IS_DEVELOPMENT));
    DogLog.setPdh(hardware.pdh);

    // Record metadata
    DogLog.log("Metadata/ProjectName", BuildConstants.MAVEN_NAME);
    DogLog.log("Metadata/RoborioSerialNumber", RobotConfig.SERIAL_NUMBER);
    DogLog.log("Metadata/RobotName", RobotConfig.get().robotName());
    DogLog.log("Metadata/BuildDate", BuildConstants.BUILD_DATE);
    DogLog.log("Metadata/GitSHA", BuildConstants.GIT_SHA);
    DogLog.log("Metadata/GitDate", BuildConstants.GIT_DATE);
    DogLog.log("Metadata/GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
        DogLog.log("Metadata/GitDirty", "All changes committed");
        break;
      case 1:
        DogLog.log("Metadata/GitDirty", "Uncomitted changes");
        break;
      default:
        DogLog.log("Metadata/GitDirty", "Unknown");
        break;
    }

    // This must be run before any commands are scheduled
    LifecycleSubsystemManager.getInstance().ready();

    SmartDashboard.putData(CommandScheduler.getInstance());

    configureBindings();
  }

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    Stopwatch.getInstance().start("Scheduler/CommandSchedulerPeriodic");
    CommandScheduler.getInstance().run();
    Stopwatch.getInstance().stop("Scheduler/CommandSchedulerPeriodic");
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = autos.getAutoCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private void configureBindings() {
    swerve.setDefaultCommand(swerve.run(() -> {
      if (DriverStation.isTeleop()) {
        swerve.driveTeleop(hardware.driverController.getLeftX(),hardware.driverController.getLeftY() , hardware.driverController.getRightX());
      }
    }));

    hardware
        .driverController
        .rightTrigger()
        .onTrue(robotCommands.confirmShotCommand())
        .onFalse(robotCommands.stopShootingCommand());
    hardware
        .driverController
        .leftTrigger()
        .onTrue(robotCommands.intakeCommand())
        .onFalse(robotCommands.stowCommand());
    hardware
        .driverController
        .rightBumper()
        .onTrue(robotCommands.passCommand())
        .onFalse(robotCommands.stopShootingCommand());
    hardware
        .driverController
        .leftBumper()
        .onTrue(robotCommands.feedingCommand())
        .onFalse(robotCommands.stopShootingCommand());

    hardware
        .operatorController
        .rightTrigger()
        .onTrue(robotCommands.ampCommand())
        .onFalse(robotCommands.stopShootingCommand());
    hardware
        .operatorController
        .leftTrigger()
        .onTrue(robotCommands.subwooferCommand())
        .onFalse(robotCommands.stowCommand());
    hardware
        .operatorController
        .x()
        .onTrue(robotCommands.outtakeCommand())
        .onFalse(robotCommands.stowCommand());
    hardware.operatorController.a().onTrue(robotCommands.stowCommand());
    hardware.operatorController.povUp().onTrue(robotCommands.climbUpCommand());
    hardware.operatorController.povDown().onTrue(robotCommands.climbDownCommand());
    hardware
        .operatorController
        .povLeft()
        .onTrue(robotCommands.unjamCommand())
        .onFalse(robotCommands.stowCommand());
  }
}
