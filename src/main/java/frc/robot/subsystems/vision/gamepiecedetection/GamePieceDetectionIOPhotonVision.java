package frc.robot.subsystems.vision.gamepiecedetection;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.vision.gamepiecedetection.GamePieceDetection.DetectionType;
import frc.robot.subsystems.vision.gamepiecedetection.GamePieceDetection.GamePiece;
import frc.robot.subsystems.vision.gamepiecedetection.GamePieceDetection.GamePieceType;
import frc.robot.subsystems.vision.gamepiecedetection.GamePieceDetectionIO.GamePieceDetectionIOInputs;

import java.util.LinkedList;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public abstract class GamePieceDetectionIOPhotonVision extends GamePieceDetectionIO {
  protected final PhotonCamera camera;
  protected final Supplier<Pose2d> robotPose;
  protected final Transform3d robotToCamera;
  protected final DetectionType detectionType;

  public GamePieceDetectionIOPhotonVision(
      String name,
      Supplier<Pose2d> robotPose,
      Transform3d robotToCamera,
      DetectionType detectionType) {
    camera = new PhotonCamera(name);
    this.robotPose = robotPose;
    this.robotToCamera = robotToCamera;
    this.detectionType = detectionType;
  }

  @Override
  public void updateInputs(GamePieceDetectionIOInputs inputs) {
    LinkedList<GamePiece> gamePieces = new LinkedList<>();
    inputs.connected = camera.isConnected();
    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        Translation2d targetTranslation =
            PhotonUtils.estimateCameraToTargetTranslation(
                PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.FLOOR_TO_MECHANISM.in(Meters) + robotToCamera.getZ(),
                    0,
                    robotToCamera.getRotation().getY(),
                    Units.degreesToRadians(target.getPitch())),
                Rotation2d.fromDegrees(target.getYaw()));
        Pose3d targetPose =
            new Pose3d(robotPose.get())
                .plus(robotToCamera)
                .plus(
                    new Transform3d(
                        PhotonUtils.estimateCameraToTarget(
                            targetTranslation, new Pose2d(), robotPose.get().getRotation())));
        gamePieces.add(
            new GamePiece(
                result.getTimestampSeconds(),
                getDetectedType(target),
                targetPose,
                target.poseAmbiguity,
                detectionType,
                getConfidence(target)));
      }
    }
    inputs.gamePieces = new GamePiece[gamePieces.size()];
    for (int i = 0; i < gamePieces.size(); i++) {
      inputs.gamePieces[i] = gamePieces.get(i);
    }
  }

  public abstract GamePieceType getDetectedType(PhotonTrackedTarget target);

  public abstract float getConfidence(PhotonTrackedTarget target);
}
