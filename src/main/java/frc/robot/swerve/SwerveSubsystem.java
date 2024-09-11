package frc.robot.swerve;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class SwerveSubsystem extends StateMachine<SwerveState> {
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

  public boolean isSlowEnoughToShoot() {
    return slowEnoughToShoot;
  }

  public SwerveSubsystem(CommandXboxController driveController) {
    super(SubsystemPriority.SWERVE, SwerveState.TELEOP);
    this.controller = driveController;
  }



  @Override
  protected void collectInputs() {
    // Module states


    // Robot relative speed

    // Field relative speed

    // If we are moving slow enough to shoot
    // TODO: Implement this
    slowEnoughToShoot = false;

    // If we are moving slow enough to feed
  }
}
