package frc.robot.subsystems.groundintake;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

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
import frc.robot.subsystems.groundintake.GroundIntakeConstants.GroundIntakeRollerConstants;
import frc.robot.util.LoggedTunableNumber;

public class GroundIntakeRollerIOTalonFX extends GroundIntakeRollerIO {

  private final StatusSignal<AngularVelocity> groundIntakeRollerVelocity;
  private final StatusSignal<Voltage> groundIntakeRollerAppliedVolts;
  private final StatusSignal<Current> groundIntakeRollerCurrent;
  private final StatusSignal<Temperature> groundIntakeRollerTemperature;
  @SuppressWarnings("unused")
  private final StatusSignal<Integer> groundIntakeRollerVersion;
  private final StatusSignal<Angle> groundIntakeRollerPosition;

  private final Debouncer groundIntakeRollerDebouncer = new Debouncer(0.5);

  public final TalonFX groundIntakeRollerMotor =
      new TalonFX(
          GroundIntakeRollerConstants.GROUNDINTAKE_ROLLER_MOTOR_ID,
          TunerConstants.DrivetrainConstants.CANBusName);
  final VelocityVoltage groundIntakeRollerVelocityRequest = new VelocityVoltage(0.0);
  final VoltageOut groundIntakeRollerOpenLoop = new VoltageOut(0.0);

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber(
          "Ground Intake Roller/kP", GroundIntakeRollerConstants.GROUNDINTAKE_ROLLER_KP);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber(
          "Ground Intake Roller/kI", GroundIntakeRollerConstants.GROUNDINTAKE_ROLLER_KI);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber(
          "Ground Intake Roller/kD", GroundIntakeRollerConstants.GROUNDINTAKE_ROLLER_KD);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Ground Intake Roller/kV", GroundIntakeRollerConstants.GROUNDINTAKE_ROLLER_KV);
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber(
          "Ground Intake Roller/kS", GroundIntakeRollerConstants.GROUNDINTAKE_ROLLER_KS);

  public GroundIntakeRollerIOTalonFX() {

    groundIntakeRollerVelocity = groundIntakeRollerMotor.getVelocity();
    groundIntakeRollerAppliedVolts = groundIntakeRollerMotor.getMotorVoltage();
    groundIntakeRollerCurrent = groundIntakeRollerMotor.getStatorCurrent();
    groundIntakeRollerTemperature = groundIntakeRollerMotor.getDeviceTemp();
    groundIntakeRollerVersion = groundIntakeRollerMotor.getVersion();
    groundIntakeRollerPosition = groundIntakeRollerMotor.getPosition();

    groundIntakeRollerMotor.getConfigurator().apply(getGroundIntakeRollerConfiguration());

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                groundIntakeRollerVelocity,
                groundIntakeRollerAppliedVolts,
                groundIntakeRollerCurrent,
                groundIntakeRollerTemperature));

    ParentDevice.optimizeBusUtilizationForAll(groundIntakeRollerMotor);
  }

  public TalonFXConfiguration getGroundIntakeRollerConfiguration() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();
    config.Slot0.kG = kS.get();
    config.Slot0.kV = kV.get();
    config.Feedback.SensorToMechanismRatio = GroundIntakeRollerConstants.GEARING;

    var currentConfig = new CurrentLimitsConfigs();
    currentConfig.StatorCurrentLimit =
        GroundIntakeRollerConstants.GROUNDINTAKE_MOTOR_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits = currentConfig;
    return config;
  }

  @Override
  public void updateInputs(GroundIntakeRollerIOInputs inputs) {
    var motor1Status =
        BaseStatusSignal.refreshAll(
            groundIntakeRollerVelocity,
            groundIntakeRollerCurrent,
            groundIntakeRollerAppliedVolts,
            groundIntakeRollerPosition,
            groundIntakeRollerTemperature);

    inputs.groundIntakeRollerConnected = groundIntakeRollerDebouncer.calculate(motor1Status.isOK());
    inputs.groundIntakeRollerVelocityRadsPerSec = groundIntakeRollerVelocity.getValue();
    inputs.groundIntakeRollerAppliedVolts = groundIntakeRollerAppliedVolts.getValue();
    inputs.groundIntakeRollerCurrentAmps = groundIntakeRollerCurrent.getValue();
    inputs.groundIntakeRollerTemperatureK = groundIntakeRollerTemperature.getValue();
    inputs.groundIntakeRollerPositionRads = groundIntakeRollerPosition.getValue();

    LoggedTunableNumber.ifChanged(
        2,
        () -> groundIntakeRollerMotor.getConfigurator().apply(getGroundIntakeRollerConfiguration()),
        kP,
        kI,
        kD,
        kV,
        kS);
  }

  @Override
  public void setGroundIntakeRollerVelocity(AngularVelocity velocity) {
    groundIntakeRollerMotor.setControl(groundIntakeRollerVelocityRequest.withVelocity(velocity));
  }

  @Override
  public void setGroundIntakeRollerOpenLoop(Voltage voltage) {
    groundIntakeRollerMotor.setControl(groundIntakeRollerOpenLoop.withOutput(voltage));
  }
}
