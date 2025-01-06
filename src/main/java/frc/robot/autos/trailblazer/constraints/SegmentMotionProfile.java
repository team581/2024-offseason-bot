package frc.robot.autos.trailblazer.constraints;

import edu.wpi.first.math.geometry.Pose2d;

public record SegmentMotionProfile(
    Pose2d endPose, double endVelocity, double currentVelocity, double maxLinearAcceleration) {}
