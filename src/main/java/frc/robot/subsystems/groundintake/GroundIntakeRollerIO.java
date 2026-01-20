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

public class GroundIntakeRollerIO {

  @AutoLog
  public static class GroundIntakeRollerIOInputs {
    public AngularVelocity groundIntakeRollerVelocity = RotationsPerSecond.of(Double.NaN);
    public Voltage groundIntakeRollerAppliedVoltage = Volts.of(Double.NaN);
    public Current groundIntakeRollerCurrent = Amps.of(Double.NaN);
    public boolean groundIntakeRollerConnected = false;
    public Temperature groundIntakeRollerTemperature = Celsius.of(Double.NaN);
    public Angle groundIntakeRollerPosition = Rotations.of(Double.NaN);
  }

  public void updateInputs(GroundIntakeRollerIOInputs inputs) {}

  public void setGroundIntakeRollerVelocity(AngularVelocity velocity) {}

  public void setGroundIntakeRollerOpenLoop(Voltage voltage) {}
}
