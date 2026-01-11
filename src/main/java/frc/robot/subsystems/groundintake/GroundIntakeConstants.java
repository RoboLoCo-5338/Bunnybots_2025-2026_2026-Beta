package frc.robot.subsystems.groundintake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.PoundSquareInches;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public final class GroundIntakeConstants {

  public static class GroundIntakeRollerConstants {
    public static final int GROUNDINTAKE_ROLLER_MOTOR_ID = 45;
    public static final Current GROUNDINTAKE_MOTOR_CURRENT_LIMIT = Amps.of(60);

    public static final double GROUNDINTAKE_ROLLER_KP = 0.5;
    public static final double GROUNDINTAKE_ROLLER_KI = 0.0;
    public static final double GROUNDINTAKE_ROLLER_KD = 0.0;
    public static final double GROUNDINTAKE_ROLLER_KS = 0.0;
    public static final double GROUNDINTAKE_ROLLER_KV = 0.0;
    public static final double GEARING = 5.0 / 3.0;

    public static final AngularVelocity GROUNDINTAKE_ROLLER_VELOCITY = RotationsPerSecond.of(-30);

    public static final AngularVelocity RESET_TOLERANCE = RotationsPerSecond.of(0.05);

    public static class GroundIntakeRollerSimConstants {
      public static final MomentOfInertia MOI = PoundSquareInches.of(158.1);
      public static final Translation3d GROUNDINTAKE_SHORT_WHEEL_ORIGIN =
          new Translation3d(0.2855, 0.13, 0.511).minus(new Translation3d(-0.084, 0.13, 0.19));
      public static final Translation3d GROUNDINTAKE_BACK_LONG_WHEEL_ORIGIN =
          new Translation3d(0.393, 0.14, 0.547).minus(new Translation3d(-0.088, 0.14, 0.193));
      public static final Translation3d GROUNDINTAKE_FRONT_LONG_WHEEL_ORIGIN =
          new Translation3d(0.489, 0.14, 0.48).minus(new Translation3d(-0.088, 0.14, 0.193));
      public static final AngularVelocity MIN_INTAKING_VELOCITY = RotationsPerSecond.of(5.0);
    }
  }

  public static class GroundIntakePivotConstants {
    public static final int GROUNDINTAKE_PIVOT_MOTOR_ID = 46;
    public static final int GROUNDINTAKE_PIVOT_ENCODER_ID = 0;
    public static final Current GROUNDINTAKE_MOTOR_CURRENT_LIMIT = Amps.of(60);

    public static final double GROUNDINTAKE_PIVOT_KP = 0.1;
    public static final double GROUNDINTAKE_PIVOT_KI = 0.0;
    public static final double GROUNDINTAKE_PIVOT_KD = 0.0;
    public static final double GROUNDINTAKE_PIVOT_KG = 0.0;
    public static final double GROUNDINTAKE_PIVOT_KV = 0.0;
    public static final double GROUNDINTAKE_PIVOT_KS = 0.0;
    public static final double GROUNDINTAKE_PIVOT_KA = 0.0;
    public static final Angle PIVOT_POSITION_TOLERANCE = Rotations.of(0.05);
    public static final double MOTOR_TO_SENSOR_GEARING = 40;
    public static final double SENSOR_TO_PIVOT_GEARING = 2;
    public static final MomentOfInertia MOI = PoundSquareInches.of(969.636);

    public static final Angle MIN_ANGLE = Radians.of(93.84 - 135);
    public static final Angle MAX_ANGLE = Radians.of(93.84);
    public static final AngularVelocity VELOCITY_RESET_TOLERANCE = RotationsPerSecond.of(0.05);
    public static final Angle POSITION_RESET_TOLERANCE = Rotations.of(0.05);
    public static final Angle SAFETY_ESCAPE_TOLERANCE = Rotations.of(0.05);
    public static final Voltage GROUND_INTAKE_PIVOT_VOLTAGE_UP = Volts.of(4);
    public static final Voltage GROUND_INTAKE_PIVOT_VOLTAGE_DOWN = Volts.of(-4);

    public static class GroundIntakePivotSimConstants {
      // Edited all values, information found on Slack
      public static final Distance LENGTH = Inches.of(12.75);
      public static final Angle STARTING_ANGLE =
          Radians.of(88); // I updated value from sim, used the same value as
      // the max angle
      public static final Translation3d GROUNDINTAKE_ORIGIN =
          new Translation3d(0.205, 0.47, 0.28).minus(new Translation3d(-0.079, 0.47, -0.01));
    }
  }
}
