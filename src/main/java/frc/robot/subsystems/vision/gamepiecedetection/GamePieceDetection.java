package frc.robot.subsystems.vision.gamepiecedetection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class GamePieceDetection extends SubsystemBase {

  private final GamePieceDetectionIO[] io;
  private final GamePieceDetectionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public GamePieceDetection(GamePieceDetectionIO... io) {
    this.io = io;

    // Initialize inputs
    this.inputs = new GamePieceDetectionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new GamePieceDetectionIOInputsAutoLogged();
    }
    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }
    List<Pose3d> allGamePiecePoses = new LinkedList<>();
    List<GamePiece> allGamePiecesAccepted = new LinkedList<>();

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      List<Pose3d> gamePiecePoses = new LinkedList<>();
      List<GamePiece> acceptedGamePieces = new LinkedList<>();

      for (GamePiece gamePiece : inputs[cameraIndex].gamePieces) {
        gamePiecePoses.add(gamePiece.pose);
        if (gamePiece.ambiguity() < VisionConstants.objectDetectionAmbiguity
            && (gamePiece.detectionType == DetectionType.Color
                || gamePiece.confidence > VisionConstants.objectDetectionConfidence)
            && gamePiece.type == GamePieceType.Lunite) {
          acceptedGamePieces.add(gamePiece);
        }
      }
      allGamePiecePoses.addAll(gamePiecePoses);
      // TODO: add logic to merge accepted game pieces
    }
    Logger.recordOutput(
        "Vision/AllGamePiecePoses",
        allGamePiecePoses.toArray(new Pose3d[allGamePiecePoses.size()]));
  }

  private static double getMagnitude(Transform3d transform3d) {
    return Math.sqrt(
        Math.pow(transform3d.getX(), 2)
            + Math.pow(transform3d.getY(), 2)
            + Math.pow(transform3d.getZ(), 2));
  }

  public static record GamePiece(
      double timestamp,
      GamePieceType type,
      Pose3d pose,
      double ambiguity,
      DetectionType detectionType,
      float confidence) {}

  public static enum GamePieceType {
    Lunite,
    Default
  }

  public static enum DetectionType {
    Color,
    Object
  }
}
