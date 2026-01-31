package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.PoundSquareInches;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.Constants;

public final class ShooterConstants {
  public static final int SHOOTER_MOTOR_1_ID = 41; // Test Values
  public static final int SHOOTER_MOTOR_2_ID = 44;
  public static final double SHOOTER_MOTOR_VELOCITY_KP =
      Constants.CURRENT_MODE == Constants.SIM_MODE ? 0.0034718 * 2 * Math.PI / 60 : 0;
  public static final double SHOOTER_MOTOR_VELOCITY_KI = 0;
  public static final double SHOOTER_MOTOR_VELOCITY_KD = 0;
  public static final double SHOOTER_MOTOR_KV = 0.016613;
  public static final double SHOOTER_MOTOR_KS = 0.012475;
  public static final double SHOOTER_MOTOR_KA = 0.0062194;

  public static final Current SHOOTER_MOTOR_CURRENT_LIMIT = Amps.of(60);
  public static final double GEARING = 1.0;

  public static final AngularVelocity SHOOTER_REVERSE_VELOCITY = RotationsPerSecond.of(-0.4);
  public static final AngularVelocity SHOOTER_LOW_GOAL_VELOCITY = RotationsPerSecond.of(0.3);
  public static final AngularVelocity SHOOTER_HIGH_GOAL_VELOCITY = RotationsPerSecond.of(0.7);
  public static final AngularVelocity SHOOTER_NO_VELOCITY = RotationsPerSecond.of(0.0);

  public static final AngularVelocity RESET_TOLERANCE = RotationsPerSecond.of(0.05);
  public static final int LASERCAN_ID = 35;

  public static final class ShooterSimConstants {
    public static final Translation3d BOTTOM_SHOOTER_ORIGIN =
        new Translation3d(-0.0225, 0.09, 0.41).minus(new Translation3d(-0.085, 0.14, 0.015));
    public static final Translation3d TOP_SHOOTER_ORIGIN =
        new Translation3d(-0.144, 0.09, 0.51).minus(new Translation3d(-0.085, 0.14, 0.015));
    public static final MomentOfInertia SHOOTER_MOI = PoundSquareInches.of(0.729044);
  }
}
