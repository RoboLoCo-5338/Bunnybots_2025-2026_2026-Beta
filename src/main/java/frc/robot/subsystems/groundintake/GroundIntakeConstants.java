package frc.robot.subsystems.groundintake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.PoundSquareInches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class GroundIntakeConstants {

  public static class GroundIntakeRollerConstants {
    public static final int GROUNDINTAKE_ROLLER_MOTOR_ID = -1;
    public static final Current GROUNDINTAKE_MOTOR_CURRENT_LIMIT = Amps.of(60);

    public static final double GROUNDINTAKE_ROLLER_KP = 0.0;
    public static final double GROUNDINTAKE_ROLLER_KI = 0.0;
    public static final double GROUNDINTAKE_ROLLER_KD = 0.0;
    public static final double GROUNDINTAKE_ROLLER_KS = 0.0;
    public static final double GROUNDINTAKE_ROLLER_KV = 0.0;
    public static final double GEARING = 5.0 / 3.0;

    public static final double GROUNDINTAKE_ROLLER_VELOCITY_MULTIPLIER = 2.0;

    public static final AngularVelocity RESET_TOLERANCE = RotationsPerSecond.of(0.05);

    public static class GroundIntakeRollerSimConstants {
      public static final MomentOfInertia MOI = PoundSquareInches.of(158.1);
    }
  }

  public static class GroundIntakePivotConstants {
    public static final int GROUNDINTAKE_PIVOT_MOTOR_ID = -1;
    public static final int GROUNDINTAKE_PIVOT_ENCODER_ID = 0;
    public static final Current GROUNDINTAKE_MOTOR_CURRENT_LIMIT = Amps.of(60);

    public static final double GROUNDINTAKE_PIVOT_KP = 0.0;
    public static final double GROUNDINTAKE_PIVOT_KI = 0.0;
    public static final double GROUNDINTAKE_PIVOT_KD = 0.0;
    public static final double GROUNDINTAKE_PIVOT_KG = 0.0;
    public static final double GROUNDINTAKE_PIVOT_KV = 0.0;
    public static final double GROUNDINTAKE_PIVOT_KS = 0.0;
    public static final double GROUNDINTAKE_PIVOT_KA = 0.0;
    public static final Angle PIVOT_POSITION_TOLERANCE = Rotations.of(0.5);
    public static final double MOTOR_TO_SENSOR_GEARING = 40;
    public static final double SENSOR_TO_PIVOT_GEARING = 2;
    public static final MomentOfInertia MOI = PoundSquareInches.of(969.636);

    public static final Angle MIN_ANGLE = Degrees.of(93.84 - 135);
    public static final Angle MAX_ANGLE = Degrees.of(93.84);
    public static final AngularVelocity VELOCITY_RESET_TOLERANCE = RotationsPerSecond.of(0.05);
    public static final Angle POSITION_RESET_TOLERANCE = Rotations.of(0.05);
    public static final Angle SAFETY_ESCAPE_TOLERANCE = Rotations.of(0.05);
    public static final double GROUND_INTAKE_PIVOT_VELOCITY_MULTIPLIER = 0.5;

    public static class GroundIntakePivotSimConstants {
      // Edited all values, information found on Slack
      public static final Distance LENGTH = Inches.of(12.75);
      public static final Angle STARTING_ANGLE =
          Degrees.of(95); // I updated value from sim, used the same value as the max angle
    }
  }
}
