package frc.robot.sim;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.dyn4j.geometry.Ellipse;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class BunnybotsLuniteOnField extends GamePieceOnFieldSimulation {
  public static final GamePieceInfo BUNNYBOTS_LUNITE_INFO =
      new GamePieceInfo(
          "Lunite",
          new Ellipse(Units.inchesToMeters(7), Units.inchesToMeters(4)),
          Inches.of(4),
          Pounds.of(3.62),
          /* We'll probably have to estimate these */
          1.8,
          4,
          0.3 /* Estimate based on very innacurate testing */);

  public BunnybotsLuniteOnField(Translation2d initialPosition) {
    super(BUNNYBOTS_LUNITE_INFO, new Pose2d(initialPosition, new Rotation2d()));
  }
}
