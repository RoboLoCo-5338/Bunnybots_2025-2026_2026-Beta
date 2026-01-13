package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.PoundSquareInches;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.sim.maplesim.BunnybotsLuniteOnField;
import frc.robot.util.ProjectileSpeedUtils;

public final class IndexerConstants {
  public static final int INDEXERID = 43;
  public static final Current INDEXER_CURRENT_LIMIT = Amps.of(60);
  public static final double INDEXER_KP = 1.5471;
  public static final double INDEXER_KI = 0;
  public static final double INDEXER_KD = 0;
  public static final double INDEXER_KS = 0.069327;
  public static final double INDEXER_KV = 1.0403;
  public static final double INDEXER_KA = 0.013842;
  public static final int LASERCAN_ID = 34;

  public static final AngularVelocity INDEXER_INTAKE_VELOCITY = RotationsPerSecond.of(3.0);
  public static final AngularVelocity INDEXER_OUTTAKE_VELOCITY = RotationsPerSecond.of(-3.0);
  public static final AngularVelocity INDEXER_NO_VELOCITY = RotationsPerSecond.of(0.0);

  public static final AngularVelocity RESET_TOLERANCE = RotationsPerSecond.of(0.05);
  // TODO: update placeholders, or else the robot will gain sentience

  public static final MomentOfInertia MOI = PoundSquareInches.of(0.304372 * 3);
  public static final Distance INDEXER_WHEEL_RADIUS = Inches.of(1.5);
  public static final double SPEED_TRANSFER_PERCENTAGE =
      ProjectileSpeedUtils.calcSpeedTransferPercentage(
          MOI, BunnybotsLuniteOnField.BUNNYBOTS_LUNITE_INFO.gamePieceMass(), INDEXER_WHEEL_RADIUS);

  public static final class IndexerSimConstants {
    public static final double GEARING = 9;

    public static final AngularVelocity MIN_INTAKING_VELOCITY = RotationsPerSecond.of(0.5);
    public static final Translation3d FRONT_INDEXER_WHEEL_ORIGIN =
        new Translation3d(0.228, 0.08, 0.295).minus(new Translation3d(-0.052, 0.08, 0.008));
    public static final Translation3d FRONT_MIDDLE_INDEXER_WHEEL_ORIGIN =
        new Translation3d(0.1125, 0.065, 0.358).minus(new Translation3d(-0.0765, 0.067, 0.056));
    public static final Translation3d BACK_MIDDLE_INDEXER_WHEEL_ORIGIN =
        new Translation3d(-0.02, 0.0725, 0.42).minus(new Translation3d(-0.117, 0.0725, 0.1026));

    public static final Translation3d BACK_INDEXER_WHEEL_ORIGIN =
        new Translation3d(-0.0448, 0.08, 0.34).minus(new Translation3d(-0.052, 0.08, 0.008));

    public static final Translation3d GEARED_INDEXER_WHEEL_ORIGIN =
        new Translation3d(-0.308, 0.2085, 0.561).minus(new Translation3d(-0.198, 0.2085, 0.131));

    public static final Distance LOWER_INDEXER_LENGTH = Inches.of(19.1868508948);
    public static final Distance INDEXER_LENGTH = Inches.of(24.8709058965);

    public static final Pose3d LUNITE_INITIAL_POSE =
        new Pose3d(0.343, 0, 0.215230, new Rotation3d());
    public static final Pose3d LUNITE_MIDDLE_POSE =
        new Pose3d(-0.070267, 0, 0.329579, new Rotation3d());
    public static final Pose3d LUNITE_FINAL_POSE =
        new Pose3d(-0.018708, 0, 0.421477, new Rotation3d());
  }
}
