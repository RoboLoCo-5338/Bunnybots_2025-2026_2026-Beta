package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class PoseUtils {

  public static Translation2d flip(Translation2d translation) {
    return new Translation2d(
        Constants.FIELD_WIDTH.in(Meters) - translation.getX(), translation.getY());
  }

  public static Translation2d vertFlip(Translation2d translation) {
    return new Translation2d(
        translation.getX(), Constants.FIELD_HEIGHT.in(Meters) - translation.getY());
  }

  public static Rotation2d flip(Rotation2d rotation) {
    return new Rotation2d(Math.PI - rotation.getRadians());
  }

  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  public static Pose2d allianceFlip(Pose2d pose, Alliance alliance) {
    return (alliance == Alliance.Red) ? flip(pose) : pose;
  }

  public static Translation2d allianceFlip(Translation2d pose, Alliance alliance) {
    return (alliance == Alliance.Red) ? flip(pose) : pose;
  }

  public static Pose2d allianceFlip(Pose2d pose) {
    return allianceFlip(pose, DriverStation.getAlliance().orElse(Alliance.Blue));
  }

  public static Translation2d allianceFlip(Translation2d pose) {
    return allianceFlip(pose, DriverStation.getAlliance().orElse(Alliance.Blue));
  }

  public static double distanceBetweenPoses(Pose2d pose1, Pose2d pose2) {
    return pose1.getTranslation().getDistance(pose2.getTranslation());
  }

  public static boolean arePosesSimilar(
      Pose2d pose1, Pose2d pose2, double linearTolerance, double angularTolerance) {
    return distanceBetweenPoses(pose1, pose2) < linearTolerance
        && MathUtil.angleModulus(pose1.getRotation().getRadians())
                - MathUtil.angleModulus(pose2.getRotation().getRadians())
            < angularTolerance;
  }

  // public static Pose2d tagRotate(Pose2d pose, int tag, Alliance alliance) {
  //   boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  //   Rotation2d rot =
  //       VisionConstants.aprilTagLayout.getTagPose(tag).get().getRotation().toRotation2d();
  //   if (isFlipped) rot = rot.plus(new Rotation2d(Math.PI));
  //   return allianceFlip(pose.rotateAround(new Translation2d(4.5, 4.03), rot));
  // }
}
