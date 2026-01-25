package frc.robot.subsystems.groundintake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public class GroundIntakePivotIO {

  @AutoLog
  public static class GroundIntakePivotIOInputs {
    public AngularVelocity odometryDrivePositiongroundIntakePivotVelocity = RotationsPerSecond.of(Double.NaN);
    public Voltage groundIntakePivotAppliedVoltage = Volts.of(Double.NaN);
    public Current groundIntakePivotCurrent = Amps.of(Double.NaN);
    public boolean groundIntakePivotConnected = false;
    public Temperature groundIntakePivotTemperature = Celsius.of(Double.NaN);
    public Angle groundIntakePivotPosition = Rotations.of(Double.NaN);
  }

  public void updateInputs(GroundIntakePivotIOInputs inputs) {}

  public void setGroundIntakePivotVelocity(AngularVelocity velocity) {}

  public void setGroundIntakePivotPosition(Angle position) {}

  public void setGroundIntakePivotOpenLoop(Voltage voltage) {}
}
