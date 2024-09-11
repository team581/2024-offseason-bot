package frc.robot.swerve;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class SwerveSubsystem extends StateMachine<SwerveState> {
  /** Max speed allowed to make a speaker shot and feeding. */
  private static final double MAX_SPEED_SHOOTING = Units.feetToMeters(0.5);
  private static final double MAX_FLOOR_SPEED_SHOOTING = Units.feetToMeters(18);

  public static final double MaxSpeed = 4.75;
  private static final double MaxAngularRate = Units.rotationsToRadians(4);
  private static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);

  private static final double leftXDeadband = 0.05;
  private static final double rightXDeadband = 0.05;
  private static final double leftYDeadband = 0.05;

  public static final Translation2d FRONT_LEFT_LOCATION =
      new Translation2d(
          CommandSwerveDrivetrain.FrontLeft.LocationX, CommandSwerveDrivetrain.FrontLeft.LocationY);
  public static final Translation2d FRONT_RIGHT_LOCATION =
      new Translation2d(
          CommandSwerveDrivetrain.FrontRight.LocationX,
          CommandSwerveDrivetrain.FrontRight.LocationY);
  public static final Translation2d BACK_LEFT_LOCATION =
      new Translation2d(
          CommandSwerveDrivetrain.BackLeft.LocationX, CommandSwerveDrivetrain.BackLeft.LocationY);
  public static final Translation2d BACK_RIGHT_LOCATION =
      new Translation2d(
          CommandSwerveDrivetrain.BackRight.LocationX, CommandSwerveDrivetrain.BackRight.LocationY);
  public static final Translation2d[] MODULE_LOCATIONS =
      new Translation2d[] {
        FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION
      };
  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(MODULE_LOCATIONS);

  private final CommandXboxController controller;

  private final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain();

  public final Pigeon2 drivetrainPigeon = drivetrain.getPigeon2();

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          // I want field-centric driving in open loop
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.03)
          .withRotationalDeadband(MaxAngularRate * 0.03);

  private final SwerveRequest.FieldCentricFacingAngle driveToAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.03);
  private final SwerveModule frontLeft = drivetrain.getModule(0);
  private final SwerveModule frontRight = drivetrain.getModule(1);
  private final SwerveModule backLeft = drivetrain.getModule(2);
  private final SwerveModule backRight = drivetrain.getModule(3);

  private boolean slowEnoughToShoot = false;
  private List<SwerveModulePosition> modulePositions;
  private ChassisSpeeds robotRelativeSpeeds;
  private ChassisSpeeds fieldRelativeSpeeds;
  private boolean slowEnoughToFeed;

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return robotRelativeSpeeds;
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return fieldRelativeSpeeds;
  }

  public boolean isSlowEnoughToShoot() {
    return slowEnoughToShoot;
  }

  public boolean isSlowEnoughToFeed() {
    return slowEnoughToFeed;
  }

  public List<SwerveModulePosition> getModulePositions(){
    return modulePositions;
  }

  public SwerveSubsystem(CommandXboxController driveController) {
    super(SubsystemPriority.SWERVE, SwerveState.TELEOP);
    this.controller = driveController;
  }



  @Override
  protected void collectInputs() {
    modulePositions = calculateModulePositions();
robotRelativeSpeeds = calculateRobotRelativeSpeeds();
fieldRelativeSpeeds = calculateFieldRelativeSpeeds();
    slowEnoughToShoot = calculateMovingSlowEnoughForSpeakerShot(robotRelativeSpeeds);
    slowEnoughToFeed = calculateMovingSlowEnoughForFloorShot(robotRelativeSpeeds);
  }

  private List<SwerveModulePosition> calculateModulePositions() {
    return List.of(
        frontLeft.getPosition(true),
        frontRight.getPosition(true),
        backLeft.getPosition(true),
        backRight.getPosition(true));
  }

  private ChassisSpeeds calculateRobotRelativeSpeeds() {
    return KINEMATICS.toChassisSpeeds(drivetrain.getState().ModuleStates);
  }

  private ChassisSpeeds calculateFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRelativeSpeeds,
        Rotation2d.fromDegrees(drivetrainPigeon.getYaw().getValueAsDouble()));
  }
  private static boolean calculateMovingSlowEnoughForSpeakerShot(ChassisSpeeds speeds) {
    double linearSpeed =
        Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));

    return linearSpeed < MAX_SPEED_SHOOTING;
  }

  private boolean calculateMovingSlowEnoughForFloorShot(ChassisSpeeds speeds) {
    double linearSpeed =
        Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));

    return linearSpeed < MAX_FLOOR_SPEED_SHOOTING;
  }
}
