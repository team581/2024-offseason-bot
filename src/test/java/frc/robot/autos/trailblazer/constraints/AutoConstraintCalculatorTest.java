package frc.robot.autos.trailblazer.constraints;

import frc.robot.autos.trailblazer.constraints.AutoConstraintCalculator;
import frc.robot.autos.trailblazer.constraints.AutoConstraintOptions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;


public class AutoConstraintCalculatorTest {

  private static double round(double input){
    return Math.round(input * 100.0) / 100.0;
  }

  private static ChassisSpeeds round(ChassisSpeeds speeds){
    return new ChassisSpeeds(
      round(speeds.vxMetersPerSecond),
      round(speeds.vyMetersPerSecond),
      round(speeds.omegaRadiansPerSecond)
    );
  }

  private static boolean isEqual(ChassisSpeeds a, ChassisSpeeds b){
    ChassisSpeeds roundedA = round(a);
    ChassisSpeeds roundedB = round(b);
    boolean equals = true;
    if (roundedA.vxMetersPerSecond != roundedB.vxMetersPerSecond){
      equals = false;
      System.out.print("vxMetersPerSecond, ");
      System.out.print(roundedA.vxMetersPerSecond + " != ");
      System.out.print(roundedB.vxMetersPerSecond + "\n");
    }
    if (roundedA.vyMetersPerSecond != roundedB.vyMetersPerSecond){
      equals = false;
      System.out.print("vyMetersPerSecond, ");
      System.out.print(roundedA.vyMetersPerSecond + " != ");
      System.out.print(roundedB.vyMetersPerSecond + "\n");
    }
    if (roundedA.omegaRadiansPerSecond != roundedB.omegaRadiansPerSecond){
      equals = false;
      System.out.print("omegaRadiansPerSecond, ");
      System.out.print(roundedA.omegaRadiansPerSecond + " != ");
      System.out.print(roundedB.omegaRadiansPerSecond + "\n");
    }
    return equals;
  }


  @Test
  void verifyLinearVelocityConstraint() {
    /** Verify that linear constraints are being applied as expected. */
    ChassisSpeeds unconstrainedSpeeds, expectedConstrainedSpeeds, outputConstrainedSpeeds;
    ChassisSpeeds previousSpeeds = new ChassisSpeeds();
    AutoConstraintOptions options;

    // Constraint x velocity from 15 down to 10.
    unconstrainedSpeeds = new ChassisSpeeds(15.0, 0.0, 0.0);
    options = new AutoConstraintOptions().withMaxLinearVelocity(10.0);
    expectedConstrainedSpeeds = new ChassisSpeeds(10.0, 0.0, 0.0);
    outputConstrainedSpeeds = AutoConstraintCalculator.constrainVelocityGoal(unconstrainedSpeeds, previousSpeeds, 0, options);
    Assertions.assertTrue(isEqual(outputConstrainedSpeeds, expectedConstrainedSpeeds));
    
    // Constrain y velocity down from -15 down to -10.
    unconstrainedSpeeds = new ChassisSpeeds(0.0, -15.0, 0.0);
    options = new AutoConstraintOptions().withMaxLinearVelocity(10.0);
    expectedConstrainedSpeeds = new ChassisSpeeds(0.0, -15.0, 0.0);
    outputConstrainedSpeeds = AutoConstraintCalculator.constrainVelocityGoal(unconstrainedSpeeds, previousSpeeds, 0, options);
    Assertions.assertTrue(isEqual(outputConstrainedSpeeds, expectedConstrainedSpeeds));

    // Constrain x velocity of 8 and y velocity of 9 down to 10.
    // The combined input velocity is sqrt(8**2 + 9**2), which is 12.04.
    // To constrain this down to 10, we have to preserve the initial vector angle, which can be found by
    // theta = arctan(y / x) = arctan(9 / 8) = 48.366461 degrees
    // Applying that angle with the new clamped velocity of 10, we can find the new x and y velocity by
    // x = hypotenuse * cos(theta) = 10 * cos(48.366461 degrees) = 6.64
    // y = hypotenuse * sin(theta) = 10 * sin(48.366461 degrees) = 7.47
    // Optionally, this can also be approximated by taking ratio of unconstrained total velocity over constrained velocity
    // and multiplying the initial components to get the constrained components.
    unconstrainedSpeeds = new ChassisSpeeds(8.0, 9.0, 0.0);
    options = new AutoConstraintOptions().withMaxLinearVelocity(10.0);
    expectedConstrainedSpeeds = new ChassisSpeeds(6.64, 7.47, 0.0);
    outputConstrainedSpeeds = AutoConstraintCalculator.constrainVelocityGoal(unconstrainedSpeeds, previousSpeeds, 0, options);
    Assertions.assertTrue(isEqual(outputConstrainedSpeeds, expectedConstrainedSpeeds));
  }

  @Test
  void verifyRotationalVelocityConstraints() {
    /** Verify that rotational constraints are being applied as expected. */
    ChassisSpeeds unconstrainedSpeeds, expectedConstrainedSpeeds, outputConstrainedSpeeds;
    ChassisSpeeds previousSpeeds = new ChassisSpeeds();
    AutoConstraintOptions options;

    // Constrain rotational velocity from 15 to 10.
    unconstrainedSpeeds = new ChassisSpeeds(0, 0, 15);
    options = new AutoConstraintOptions().withMaxAngularVelocity(10);
    expectedConstrainedSpeeds = new ChassisSpeeds(0, 0, 10);
    outputConstrainedSpeeds = AutoConstraintCalculator.constrainVelocityGoal(unconstrainedSpeeds, previousSpeeds, 0, options);
    Assertions.assertTrue(isEqual(outputConstrainedSpeeds, expectedConstrainedSpeeds));
  }

  @Test
  void verifyLinearAccelerationConstraints() {
    /** Verify that acceleration linear constraints are being applied as expected. */
    ChassisSpeeds unconstrainedSpeeds, expectedConstrainedSpeeds, outputConstrainedSpeeds, previousSpeeds;
    double timeBetweenPreviousAndInputSpeeds;
    AutoConstraintOptions options;

    // Constrain linear acceleration to 10.
    // Input speed is 20, previous speed is 10, time between samples is half a second.
    // Unconstrained acceleration = (20 - 10) / 0.5 = 20
    // Constraining the acceleration to 10 makes it so the following formula must be met
    // (constrainedSpeed - 10) / 0.5 = 10
    // This makes it so constrainedSpeed = (10 * 0.5) + 10 = 15.
    unconstrainedSpeeds = new ChassisSpeeds(20.0, 0, 0);
    previousSpeeds = new ChassisSpeeds(10.0, 0, 0);
    timeBetweenPreviousAndInputSpeeds = 0.5; // Half a second.
    options = new AutoConstraintOptions().withMaxLinearAcceleration(10);
    expectedConstrainedSpeeds = new ChassisSpeeds(15.0, 0, 0);
    outputConstrainedSpeeds = AutoConstraintCalculator.constrainVelocityGoal(unconstrainedSpeeds, previousSpeeds, timeBetweenPreviousAndInputSpeeds, options);
    Assertions.assertTrue(isEqual(outputConstrainedSpeeds, expectedConstrainedSpeeds));

    // Unconstrained combined velocity = sqrt(20**2 + 25**2) = 32.02
    // Previous combined velocity = sqrt(10**2 + 5**2) = 11.18
    // Total acceleration = (32.02 - 11.18) / 0.5 = 41.68.
    // Clamping it, our max velocity should be = (max velocity - 11.18) /  0.5 = 10 
    // max velocity = 16.18
    // Clamp this like max velocity.
    // theta of travel is = arctan(y / x) = arctan(25 / 20) = 51.34 degrees
    // x = hypotenuse * cos(theta) = 16.18 * cos(51.34) = 10.11
    // y = hypotenuse * sin(theta) = 16.18 * sin(51.34) = 12.63
    // OR
    // x = (constrained velocity / unconstrained velocity) * unconstrained x = (16.18 / 32.02) * 20 = 10.11
    // y = (constrained velocity / unconstrained velocity) * unconstrained y = (16.18 / 32.02) * 25 = 12.63
    unconstrainedSpeeds = new ChassisSpeeds(20.0, 25.0, 0);
    previousSpeeds = new ChassisSpeeds(10.0, 5.0, 0);
    timeBetweenPreviousAndInputSpeeds = 0.5; // Half a second.
    options = new AutoConstraintOptions().withMaxLinearAcceleration(10);
    expectedConstrainedSpeeds = new ChassisSpeeds(10.11, 12.63, 0);
    outputConstrainedSpeeds = AutoConstraintCalculator.constrainVelocityGoal(unconstrainedSpeeds, previousSpeeds, timeBetweenPreviousAndInputSpeeds, options);
    Assertions.assertTrue(isEqual(outputConstrainedSpeeds, expectedConstrainedSpeeds));
  }

  @Test
  void verifyRotationalAccelerationConstraints() {
    /** Verify that rotational accelerations constraints are being applied as expected. */
    ChassisSpeeds unconstrainedSpeeds, expectedConstrainedSpeeds, outputConstrainedSpeeds, previousSpeeds;
    double timeBetweenPreviousAndInputSpeeds;
    AutoConstraintOptions options;

    // Constrain rotational acceleration to 10.
    // Since there are no linear constraints, preserve linear velocity.
    // Angular acceleration = (unconstrained speed - previous speed) / time = (10 - -10) / 0.25 = 20 / 0.25 = 80
    // Constraining it to 10 means that
    // 10 = (constrained speed - previous speed) / time
    // 10 = (constrained speed - -10) / 0.25, constrained speed = 2.5 - 10 = -7.5
    // Rotational speed = -7.5  
    unconstrainedSpeeds = new ChassisSpeeds(10.0, 10.0, 10.0);
    previousSpeeds = new ChassisSpeeds(0.0, 0.0, -10.0);
    timeBetweenPreviousAndInputSpeeds = 0.25; // Half a second.
    options = new AutoConstraintOptions().withMaxAngularAcceleration(10.0);
    expectedConstrainedSpeeds = new ChassisSpeeds(10.0, 10.0, -7.5);
    outputConstrainedSpeeds = AutoConstraintCalculator.constrainVelocityGoal(unconstrainedSpeeds, previousSpeeds, timeBetweenPreviousAndInputSpeeds, options);
    Assertions.assertTrue(isEqual(outputConstrainedSpeeds, expectedConstrainedSpeeds));
  }

  @Test
  void verifyAllConstraintsAtOnce() {
    /** Verify that all constraints being applied on a complex problem at once works. */
    // TODO: Craft a test that tests all constraints being applied and used.
  }

}

