package frc.robot.subsystems.vision.gamepiecedetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.gamepiecedetection.GamePieceDetection.DetectionType;
import frc.robot.subsystems.vision.gamepiecedetection.GamePieceDetection.GamePieceType;
import java.util.function.Supplier;
import org.photonvision.targeting.PhotonTrackedTarget;

public class GamePieceColorDetectionIOPhotonVision extends GamePieceDetectionIOPhotonVision {
  GamePieceType detectedType;

  public GamePieceColorDetectionIOPhotonVision(
      String name,
      Supplier<Pose2d> robotPose,
      Transform3d robotToCamera,
      GamePieceType detectedType) {
    super(name, robotPose, robotToCamera, DetectionType.Color);
    this.detectedType = detectedType;
  }

  @Override
  public GamePieceType getDetectedType(PhotonTrackedTarget target) {
    return detectedType;
  }

  @Override
  public float getConfidence(PhotonTrackedTarget target) {
    return -1; // no confidence in color detection
  }
}
