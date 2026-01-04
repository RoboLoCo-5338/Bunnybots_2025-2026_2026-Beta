package frc.robot.sim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.PoseUtils;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

public class BunnybotsStarSpire implements SimulatedArena.Simulatable {
  private final Arena2025Bunnybots arena;
  private HumanBehavior humanBehavior = HumanBehavior.OFF;
  private final Pose2d sourcePosition;
  private double previousThrowTimeSeconds = 0;

  public BunnybotsStarSpire(
      Arena2025Bunnybots arena,
      Alliance alliance,
      SpirePosition position,
      HumanBehavior humanBehavior) {
    this.arena = arena;
    this.humanBehavior = humanBehavior;
    this.sourcePosition = PoseUtils.allianceFlip(position.pose, alliance);
  }

  @Override
  /*
   * TODO: This causes a weird race condition where sometimes multiple game pieces
   * are spawned at the same time due to the frequency of simulationSubTick
   */
  public void simulationSubTick(int subTickNum) {
    // lastCallTimestap = Timer.getFPGATimestamp();
    if (!DriverStation.isTeleopEnabled()
        || humanBehavior == HumanBehavior.OFF
        || Timer.getFPGATimestamp() - previousThrowTimeSeconds < 2) {
      return;
    }

    int c = 0;

    for (GamePieceOnFieldSimulation gamePiece : arena.gamePiecesOnField())
      if (gamePiece.getPoseOnField().getTranslation().getDistance(sourcePosition.getTranslation())
          < 1) c++;

    for (GamePieceProjectile gamePiece : arena.gamePieceLaunched())
      if (gamePiece
              .getPose3d()
              .getTranslation()
              .toTranslation2d()
              .getDistance(sourcePosition.getTranslation())
          < 1) c++;

    if (c >= 3) {
      return;
    }

    previousThrowTimeSeconds = Timer.getFPGATimestamp();

    arena.addGamePieceProjectile(
        new BunnybotsLuniteOnFly(
            sourcePosition.getTranslation(),
            new Translation2d(0, 0),
            new ChassisSpeeds(),
            sourcePosition.getRotation(),
            Inches.of(24.448),
            MetersPerSecond.of(randomThrowSpeed()),
            Degrees.of(-26.249026)));
  }

  public void setHumanBehavior(HumanBehavior humanBehavior) {
    this.humanBehavior = humanBehavior;
  }

  public static double randomThrowSpeed() {
    return Math.random() * 1.25; // 1 to 3 m/s
    // TODO: Check if this is a good range
  }

  public void resetTime() {
    previousThrowTimeSeconds = 0;
  }

  public static enum HumanBehavior {
    OFF,
    ON
  }

  public static enum SpirePosition {
    NEAR(new Pose2d(0.2, 6.940, Rotation2d.fromDegrees(0))),
    FAR(new Pose2d(1.920, 8, Rotation2d.fromDegrees(-90)));

    public final Pose2d pose;

    SpirePosition(Pose2d pose) {
      this.pose = pose;
    }
  }
}
