package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public class IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public AngularVelocity indexerVelocityRadPerSec = RotationsPerSecond.of(Double.NaN);
    public Voltage indexerAppliedVolts = Volts.of(Double.NaN);
    public Current indexerCurrentAmps = Amps.of(Double.NaN);
    public boolean indexerConnected = false;
    public Temperature indexerTemperatureK = Celsius.of(Double.NaN);
    public Angle indexerPositionRads = Rotations.of(Double.NaN);
    public Distance indexerDistanceM = Millimeters.of(Double.NaN);
  }

  public void updateInputs(IndexerIOInputs inputs) {}

  public void setIndexerVelocity(AngularVelocity velocity) {}

  public void indexerOpenLoop(Voltage voltage) {}
}
