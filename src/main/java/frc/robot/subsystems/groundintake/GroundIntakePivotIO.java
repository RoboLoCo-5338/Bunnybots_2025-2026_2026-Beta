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
    public AngularVelocity groundIntakePivotVelocityRadsPerSec = RotationsPerSecond.of(Double.NaN);
    public Voltage groundIntakePivotAppliedVolts = Volts.of(Double.NaN);
    public Current groundIntakePivotCurrentAmps = Amps.of(Double.NaN);
    public boolean groundIntakePivotConnected = false;
    public Temperature groundIntakePivotTemperatureK = Celsius.of(Double.NaN);
    public Angle groundIntakePivotPositionRads = Rotations.of(Double.NaN);
  }

  public void updateInputs(GroundIntakePivotIOInputs inputs) {}

  public void setGroundIntakePivotVelocity(AngularVelocity velocity) {}

  public void setGroundIntakePivotPosition(Angle position) {}

  public void setGroundIntakePivotOpenLoop(Voltage voltage) {}
}
