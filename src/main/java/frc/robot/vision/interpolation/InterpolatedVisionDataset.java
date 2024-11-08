package frc.robot.vision.interpolation;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/** A interpolated vision data set for each field, for all cameras. */
public enum InterpolatedVisionDataset {
  HOME(
      new CameraDataset(
          List.of(
              new VisionInterpolationData(
                  new Translation2d(15.2245, 5.522), new Translation2d(15.201, 5.702), "SUBWOOFER"),
              new VisionInterpolationData(
                  new Translation2d(12.9895, 5.522),
                  new Translation2d(12.965, 5.655),
                  "PODIUM_SPEAKER_INTERSECTION")),
          List.of()),
      new CameraDataset(
          List.of(
              new VisionInterpolationData(
                  new Translation2d(15.2245, 5.522), new Translation2d(15.202, 5.713), "SUBWOOFER"),
              new VisionInterpolationData(
                  new Translation2d(12.9895, 5.522),
                  new Translation2d(12.87, 5.581),
                  "PODIUM_SPEAKER_INTERSECTION")),
          List.of())),
  // bellarmine is not tested
  BELLARMINE(
      new CameraDataset(
          List.of(
              new VisionInterpolationData(
                  new Translation2d(15.2245, 5.522), new Translation2d(15.125, 5.581), "SUBWOOFER"),
              new VisionInterpolationData(
                  new Translation2d(13.0745, 5.522),
                  new Translation2d(13.103, 5.560),
                  "PODIUM_SPEAKER_INTERSECTION"),
              new VisionInterpolationData(
                  new Translation2d(11.059, 6.842),
                  new Translation2d(11.18, 6.932),
                  "WING_LINE_MIDDLE"),
              new VisionInterpolationData(
                  new Translation2d(13.799, 4.202),
                  new Translation2d(13.67, 4.106),
                  "FRONT_PODIUM_MIDDLE")),
          List.of()),
      new CameraDataset(
          List.of(
              new VisionInterpolationData(
                  new Translation2d(15.2245, 5.522), new Translation2d(15.125, 5.581), "SUBWOOFER"),
              new VisionInterpolationData(
                  new Translation2d(13.0745, 5.522),
                  new Translation2d(13.103, 5.560),
                  "PODIUM_SPEAKER_INTERSECTION"),
              new VisionInterpolationData(
                  new Translation2d(11.059, 6.842),
                  new Translation2d(11.18, 6.932),
                  "WING_LINE_MIDDLE"),
              new VisionInterpolationData(
                  new Translation2d(13.799, 4.202),
                  new Translation2d(13.67, 4.106),
                  "FRONT_PODIUM_MIDDLE")),
          List.of())),
  MADTOWN(
      new CameraDataset(
          List.of(
              new VisionInterpolationData(
                  new Translation2d(15.2245, 5.522), new Translation2d(999, 999), "SUBWOOFER"),
              new VisionInterpolationData(
                  new Translation2d(13.0745, 5.522),
                  new Translation2d(999, 999),
                  "PODIUM_SPEAKER_INTERSECTION"),
              new VisionInterpolationData(
                  new Translation2d(11.059, 6.842),
                  new Translation2d(999, 999),
                  "WING_LINE_MIDDLE"),
              new VisionInterpolationData(
                  new Translation2d(13.799, 4.202),
                  new Translation2d(999, 999),
                  "FRONT_PODIUM_MIDDLE")),
          List.of()),
      new CameraDataset(
          List.of(
              new VisionInterpolationData(
                  new Translation2d(15.2245, 5.522), new Translation2d(999, 999), "SUBWOOFER"),
              new VisionInterpolationData(
                  new Translation2d(13.0745, 5.522),
                  new Translation2d(999, 999),
                  "PODIUM_SPEAKER_INTERSECTION"),
              new VisionInterpolationData(
                  new Translation2d(11.059, 6.842),
                  new Translation2d(999, 999),
                  "WING_LINE_MIDDLE"),
              new VisionInterpolationData(
                  new Translation2d(13.799, 4.202),
                  new Translation2d(999, 999),
                  "FRONT_PODIUM_MIDDLE")),
          List.of()));

  public final CameraDataset leftSet;
  public final CameraDataset rightSet;

  InterpolatedVisionDataset(CameraDataset left, CameraDataset right) {
    this.leftSet = left;
    this.rightSet = right;
  }
}
