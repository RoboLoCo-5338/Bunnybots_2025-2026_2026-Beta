package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Millimeters;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LoggedTunableNumber;

public class IndexerIOTalonFX extends IndexerIO {
  private final StatusSignal<AngularVelocity> indexerVelocity;
  private final StatusSignal<Voltage> indexerAppliedVolts;
  private final StatusSignal<Current> indexerCurrent;
  private final StatusSignal<Temperature> indexerTemperature;
  @SuppressWarnings("unused")
  private final StatusSignal<Integer> indexerVersion;
  private final StatusSignal<Angle> indexerPosition;

  private final Debouncer indexerDebouncer = new Debouncer(0.5);

  private final LaserCan lcIndexer;

  public final TalonFX indexerMotor =
      new TalonFX(IndexerConstants.INDEXERID, TunerConstants.DrivetrainConstants.CANBusName);
  final VelocityVoltage indexerVelocityRequest = new VelocityVoltage(0.0);
  final VoltageOut indexerOpenLoop = new VoltageOut(0.0);

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Indexer kP", IndexerConstants.INDEXER_KP);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Indexer kI", IndexerConstants.INDEXER_KI);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Indexer kD", IndexerConstants.INDEXER_KD);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Indexer kV", IndexerConstants.INDEXER_KV);
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Indexer kS", IndexerConstants.INDEXER_KS);
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Indexer kA", IndexerConstants.INDEXER_KA);

  public IndexerIOTalonFX() {
    indexerVelocity = indexerMotor.getVelocity();
    indexerAppliedVolts = indexerMotor.getMotorVoltage();
    indexerCurrent = indexerMotor.getStatorCurrent();
    indexerTemperature = indexerMotor.getDeviceTemp();
    indexerVersion = indexerMotor.getVersion();
    indexerPosition = indexerMotor.getPosition();

    indexerMotor.getConfigurator().apply(getIndexerConfiguration());

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, indexerVelocity, indexerAppliedVolts, indexerCurrent, indexerTemperature));

    ParentDevice.optimizeBusUtilizationForAll(indexerMotor);

    lcIndexer = new LaserCan(IndexerConstants.LASERCAN_ID);

    try {
      lcIndexer.setRangingMode(LaserCan.RangingMode.SHORT);
      lcIndexer.setRegionOfInterest(new LaserCan.RegionOfInterest(2, 2, 2, 2)); // TODO: update
      lcIndexer.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (Exception e) {
      System.out.println("Error: " + e);
    }
  }

  public TalonFXConfiguration getIndexerConfiguration() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();
    config.Slot0.kV = kV.get();
    config.Slot0.kS = kS.get();
    config.Slot0.kA = kA.get();

    config.Feedback.SensorToMechanismRatio = IndexerConstants.IndexerSimConstants.GEARING;

    var currentConfig = new CurrentLimitsConfigs();
    currentConfig.StatorCurrentLimit = IndexerConstants.INDEXER_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits = currentConfig;
    return config;
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    var motor1Status =
        BaseStatusSignal.refreshAll(
            indexerVelocity,
            indexerCurrent,
            indexerAppliedVolts,
            indexerPosition,
            indexerTemperature);

    inputs.indexerConnected = indexerDebouncer.calculate(motor1Status.isOK());
    Measurement m1 = lcIndexer.getMeasurement();
    if (m1 == null) {
      // LaserCan did not return a measurement
      inputs.indexerDistanceM = Millimeters.of(-1);
    } else if (m1.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      inputs.indexerDistanceM = Millimeters.of(m1.distance_mm);
    } else if (m1.status == LaserCan.LASERCAN_STATUS_WEAK_SIGNAL) {
      // LaserCan returned a weak signal
      inputs.indexerDistanceM = Millimeters.of(-1);
    }
    inputs.indexerVelocityRadPerSec = indexerVelocity.getValue();
    inputs.indexerAppliedVolts = indexerAppliedVolts.getValue();
    inputs.indexerCurrentAmps = indexerCurrent.getValue();
    inputs.indexerTemperatureK = indexerTemperature.getValue();
    inputs.indexerPositionRads = indexerPosition.getValue();

    LoggedTunableNumber.ifChanged(
        0,
        () -> indexerMotor.getConfigurator().apply(getIndexerConfiguration()),
        kP,
        kI,
        kD,
        kV,
        kS,
        kA);
  }

  @Override
  public void setIndexerVelocity(AngularVelocity velocity) {
    indexerMotor.setControl(indexerVelocityRequest.withVelocity(velocity));
  }

  @Override
  public void indexerOpenLoop(Voltage voltage) {
    indexerMotor.setControl(indexerOpenLoop.withOutput(voltage));
  }
}
