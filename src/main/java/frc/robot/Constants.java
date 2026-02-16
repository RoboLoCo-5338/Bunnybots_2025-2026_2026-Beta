package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final Mode SIM_MODE = Mode.SIM;
  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;
  public static final Distance ROBOT_LENGTH = Inches.of(33.250000);
  public static final Distance FLOOR_TO_MECHANISM = Inches.of(8);
  public static final boolean TUNING = true;

  public static final Distance FIELD_WIDTH = Meters.of(16.4592);
  public static final Distance FIELD_HEIGHT = Meters.of(8.2296);

  public static final MomentOfInertiaUnit PoundSquareInch =
      Pounds.mult(InchesPerSecond).mult(Inches).per(RadiansPerSecond);
  public static final MomentOfInertiaUnit PoundSquareInches = PoundSquareInch;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
