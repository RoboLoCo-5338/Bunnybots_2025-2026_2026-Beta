package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.PoundSquareInches;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class IndexerConstants {
  public static final int INDEXERID = 0;
  public static final Current INDEXER_CURRENT_LIMIT = Amps.of(60);
  public static final double INDEXER_KP = 1.5471;
  public static final double INDEXER_KI = 0;
  public static final double INDEXER_KD = 0;
  public static final double INDEXER_KS = 0.0019606;
  public static final double INDEXER_KV = 1.5398;
  public static final double INDEXER_KA = 0.013842;
  public static final int LASERCAN_ID = -1;

  public static final AngularVelocity INDEXER_INTAKE_VELOCITY = RotationsPerSecond.of(1.0);
  public static final AngularVelocity INDEXER_OUTTAKE_VELOCITY = RotationsPerSecond.of(-1.0);
  public static final AngularVelocity INDEXER_NO_VELOCITY = RotationsPerSecond.of(0.0);

  public static final AngularVelocity RESET_TOLERANCE = RotationsPerSecond.of(0.05);
  // TODO: update placeholders, or else the robot will gain sentience

  public static final class IndexerSimConstants {
    public static final MomentOfInertia MOI = PoundSquareInches.of(0.304372 * 3);
    public static final double GEARING = 9;
  }
}
