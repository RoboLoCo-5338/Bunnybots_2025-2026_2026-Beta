package frc.robot.subsystems.vision.gamepiecedetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.gamepiecedetection.GamePieceDetection.DetectionType;
import frc.robot.subsystems.vision.gamepiecedetection.GamePieceDetection.GamePieceType;
import java.util.function.Supplier;
import org.photonvision.targeting.PhotonTrackedTarget;

public class GamePieceObjectDetectionIOPhotonVision extends GamePieceDetectionIOPhotonVision {
  public GamePieceObjectDetectionIOPhotonVision(
      String name, Supplier<Pose2d> robotPose, Transform3d robotToCamera) {
    super(name, robotPose, robotToCamera, DetectionType.Object);
  }

  @Override
  public GamePieceType getDetectedType(PhotonTrackedTarget target) {
    return switch (target.objDetectId) {
      case 0 -> GamePieceType.Lunite; // FIGURE OUT WHAT THE ID IS
      default -> GamePieceType.Default;
    };
  }

  @Override
  public float getConfidence(PhotonTrackedTarget target) {
    return target.objDetectConf;
  }
}
