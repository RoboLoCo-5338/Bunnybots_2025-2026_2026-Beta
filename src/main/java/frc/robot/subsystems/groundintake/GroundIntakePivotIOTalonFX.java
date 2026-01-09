package frc.robot.subsystems.groundintake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.groundintake.GroundIntakeConstants.GroundIntakePivotConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class GroundIntakePivotIOTalonFX extends GroundIntakePivotIO {

  private final StatusSignal<Angle> groundIntakePivotPosition;
  private final StatusSignal<AngularVelocity> groundIntakePivotVelocity;
  private final StatusSignal<Voltage> groundIntakePivotAppliedVolts;
  private final StatusSignal<Current> groundIntakePivotCurrent;
  private final StatusSignal<Temperature> groundIntakePivotTemperature;

  @SuppressWarnings("unused")
  private final StatusSignal<Integer> groundIntakePivotVersion;

  private final Debouncer groundIntakePivotDebouncer = new Debouncer(0.5);
  protected final REVThroughBoreEncoder groundIntakePivotEncoder;

  public final TalonFX groundIntakePivotMotor =
      new TalonFX(
          GroundIntakeConstants.GroundIntakePivotConstants.GROUNDINTAKE_PIVOT_MOTOR_ID,
          TunerConstants.DrivetrainConstants.CANBusName);
  final VelocityVoltage groundIntakePivotVelocityRequest = new VelocityVoltage(0.0);
  final PositionVoltage groundIntakePivotPositionRequest = new PositionVoltage(0.0);
  final VoltageOut groundIntakePivotOpenLoop = new VoltageOut(0.0);

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber(
          "Ground Intake Pivot/kP", GroundIntakePivotConstants.GROUNDINTAKE_PIVOT_KP);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber(
          "Ground Intake Pivot/kI", GroundIntakePivotConstants.GROUNDINTAKE_PIVOT_KI);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber(
          "Ground Intake Pivot/kD", GroundIntakePivotConstants.GROUNDINTAKE_PIVOT_KD);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Ground Intake Pivot/kV", GroundIntakePivotConstants.GROUNDINTAKE_PIVOT_KS);
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber(
          "Ground Intake Pivot/kS", GroundIntakePivotConstants.GROUNDINTAKE_PIVOT_KS);
  private final LoggedTunableNumber kG =
      new LoggedTunableNumber(
          "Ground Intake Pivot/kG", GroundIntakePivotConstants.GROUNDINTAKE_PIVOT_KG);
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber(
          "Ground Intake Pivot/kA", GroundIntakePivotConstants.GROUNDINTAKE_PIVOT_KA);

  public GroundIntakePivotIOTalonFX() {
    groundIntakePivotEncoder =
        new REVThroughBoreEncoder(GroundIntakePivotConstants.GROUNDINTAKE_PIVOT_ENCODER_ID);
    groundIntakePivotEncoder.setInverted(true);
    groundIntakePivotPosition = groundIntakePivotMotor.getPosition();
    groundIntakePivotVelocity = groundIntakePivotMotor.getVelocity();
    groundIntakePivotAppliedVolts = groundIntakePivotMotor.getMotorVoltage();
    groundIntakePivotCurrent = groundIntakePivotMotor.getStatorCurrent();
    groundIntakePivotTemperature = groundIntakePivotMotor.getDeviceTemp();
    groundIntakePivotVersion = groundIntakePivotMotor.getVersion();
    configureMotor();
    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                groundIntakePivotPosition,
                groundIntakePivotVelocity,
                groundIntakePivotAppliedVolts,
                groundIntakePivotCurrent,
                groundIntakePivotTemperature));

    ParentDevice.optimizeBusUtilizationForAll(groundIntakePivotMotor);
  }

  private void configureMotor() {
    groundIntakePivotMotor.getConfigurator().apply(getGroundIntakePivotConfiguration());
    groundIntakePivotMotor.setPosition(
        Rotations.of(
            MathUtil.inputModulus(
                groundIntakePivotEncoder.get() / GroundIntakePivotConstants.SENSOR_TO_PIVOT_GEARING,
                -0.5,
                0.5)));
  }

  private TalonFXConfiguration getGroundIntakePivotConfiguration() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();
    config.Slot0.kG = kG.get();
    config.Slot0.kV = kV.get();
    config.Slot0.kS = kS.get();
    config.Slot0.kA = kA.get();
    config.Feedback.SensorToMechanismRatio =
        GroundIntakePivotConstants.SENSOR_TO_PIVOT_GEARING
            * GroundIntakePivotConstants.MOTOR_TO_SENSOR_GEARING;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(Units.radiansToRotations(90));
    var currentConfig = new CurrentLimitsConfigs();
    currentConfig.StatorCurrentLimit =
        GroundIntakePivotConstants.GROUNDINTAKE_MOTOR_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits = currentConfig;
    return config;
  }

  @Override
  public void updateInputs(GroundIntakePivotIOInputs inputs) {
    var motor2Status =
        BaseStatusSignal.refreshAll(
            groundIntakePivotPosition,
            groundIntakePivotVelocity,
            groundIntakePivotCurrent,
            groundIntakePivotAppliedVolts,
            groundIntakePivotTemperature);

    inputs.groundIntakePivotConnected = groundIntakePivotDebouncer.calculate(motor2Status.isOK());
    inputs.groundIntakePivotVelocityRadsPerSec = groundIntakePivotVelocity.getValue();
    inputs.groundIntakePivotAppliedVolts = groundIntakePivotAppliedVolts.getValue();
    inputs.groundIntakePivotCurrentAmps = groundIntakePivotCurrent.getValue();
    inputs.groundIntakePivotTemperatureK = groundIntakePivotTemperature.getValue();
    inputs.groundIntakePivotPositionRads = groundIntakePivotPosition.getValue();
    Logger.recordOutput(
        "GroundIntake/Pivot/PivotEncoderPositionRadians",
        Rotations.of(
            groundIntakePivotEncoder.get() / GroundIntakePivotConstants.SENSOR_TO_PIVOT_GEARING));

    LoggedTunableNumber.ifChanged(1, () -> configureMotor(), kP, kI, kD, kV, kS, kA);
  }

  @Override
  public void setGroundIntakePivotVelocity(AngularVelocity velocity) {
    groundIntakePivotMotor.setControl(groundIntakePivotVelocityRequest.withVelocity(velocity));
  }

  @Override
  public void setGroundIntakePivotPosition(Angle position) {
    groundIntakePivotMotor.setControl(groundIntakePivotPositionRequest.withPosition(position));
  }

  @Override
  public void setGroundIntakePivotOpenLoop(Voltage voltage) {
    groundIntakePivotMotor.setControl(groundIntakePivotOpenLoop.withOutput(voltage));
  }
}
