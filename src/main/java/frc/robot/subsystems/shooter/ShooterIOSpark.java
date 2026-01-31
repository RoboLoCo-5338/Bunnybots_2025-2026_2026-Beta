package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SparkUtil;
import java.util.function.DoubleSupplier;

public class ShooterIOSpark extends ShooterIO {

  private final RelativeEncoder shooterEncoder;

  private final Debouncer shooter1ConnectedDebouncer = new Debouncer(0.5);

  private final SparkClosedLoopController shooterClosedLoopController;
  private final SparkUtil sparkUtil = new SparkUtil();

  SparkFlex shooterMotor;

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.SHOOTER_MOTOR_KS,
          ShooterConstants.SHOOTER_MOTOR_KV,
          ShooterConstants.SHOOTER_MOTOR_KA);
  LaserCan laserCan = new LaserCan(ShooterConstants.LASERCAN_ID);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter/kP", ShooterConstants.SHOOTER_MOTOR_VELOCITY_KP);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Shooter/kI", ShooterConstants.SHOOTER_MOTOR_VELOCITY_KI);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Shooter/kD", ShooterConstants.SHOOTER_MOTOR_VELOCITY_KD);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Shooter/kV", ShooterConstants.SHOOTER_MOTOR_KV);
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Shooter/kS", ShooterConstants.SHOOTER_MOTOR_KS);

  public ShooterIOSpark(int shooterNum) {
    shooterMotor =
        new SparkFlex(
            shooterNum == 1
                ? ShooterConstants.SHOOTER_MOTOR_1_ID
                : ShooterConstants.SHOOTER_MOTOR_2_ID,
            MotorType.kBrushless);
    shooterEncoder = shooterMotor.getEncoder();

    sparkUtil.tryUntilOk(
        shooterMotor,
        5,
        () ->
            shooterMotor.configure(
                getShooterConfig(),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    shooterClosedLoopController = shooterMotor.getClosedLoopController();

    try {
      laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      laserCan.setRegionOfInterest(
          new LaserCan.RegionOfInterest(2, 2, 2, 2)); // TODO: needs to be changed
      laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (Exception e) {
      System.out.println("Error: " + e);
    }
  }

  /**
   * Gets the configuration used for the Talon FX motor controllers of the shooter subsystem.
   *
   * <p>This method returns a Talon FX configuration with the following settings:
   *
   * <ul>
   *   <li>Neutral mode: Brake
   *   <li>Gravity type: Shooter cosine
   *   <li>Feedback device: Integrated sensor
   *   <li>kP: {@link ShooterConstants#SHOOTER_MOTOR_kP}
   *   <li>kI: {@link ShooterConstants#SHOOTER_MOTOR_kI}
   *   <li>kD: {@link ShooterConstants#SHOOTER_MOTOR_kD}
   *   <li>kV: {@link ShooterConstants#SHOOTER_MOTOR_kV}
   *   <li>kV: {@link ShooterConstants#SHOOTER_MOTOR_kS}
   *   <li>Current limit: 60A (CHANGE THIS VALUE OTHERWISE TORQUE MAY BE LIMITED/TOO HIGH)
   * </ul>
   *
   * <p>These values may need to be changed based on the actual robot hardware and the desired
   * behavior of the elevator.
   *
   * @return the configuration used for the Talon FX motor controllers of the shooter subsystem
   */
  public SparkFlexConfig getShooterConfig() {
    SparkFlexConfig shooterConfig = new SparkFlexConfig();

    shooterConfig.closedLoop.pid(kP.get(), kI.get(), kD.get());
    shooterConfig
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .smartCurrentLimit((int) (ShooterConstants.SHOOTER_MOTOR_CURRENT_LIMIT.in(Amps)))
        .voltageCompensation(12.0);

    shooterConfig.absoluteEncoder.velocityConversionFactor(ShooterConstants.GEARING);
    shooterConfig.absoluteEncoder.positionConversionFactor(ShooterConstants.GEARING);

    shooterConfig.idleMode(IdleMode.kCoast);

    return shooterConfig;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    sparkUtil.sparkStickyFault = false;

    sparkUtil.ifOk(
        shooterMotor,
        shooterEncoder::getPosition,
        (value) -> inputs.shooterPositionRads = Rotations.of(value * ShooterConstants.GEARING));
    sparkUtil.ifOk(
        shooterMotor,
        shooterEncoder::getVelocity,
        (value) -> inputs.shooterVelocityRadPerSec = RPM.of(value * ShooterConstants.GEARING));
    sparkUtil.ifOk(
        shooterMotor,
        new DoubleSupplier[] {shooterMotor::getAppliedOutput, shooterMotor::getBusVoltage},
        (values) -> inputs.shooterAppliedVolts = Volts.of(values[0] * values[1]));
    sparkUtil.ifOk(
        shooterMotor,
        shooterMotor::getOutputCurrent,
        (value) -> inputs.shooterCurrentAmps = Amps.of(value));
    sparkUtil.ifOk(
        shooterMotor,
        shooterMotor::getMotorTemperature,
        (value) -> inputs.shooterTemperatureK = Celsius.of(value));

    LoggedTunableNumber.ifChanged(
        9,
        () ->
            shooterMotor.configure(
                getShooterConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters),
        kP,
        kI,
        kD);
    LoggedTunableNumber.ifChanged(
        5, () -> feedforward = new SimpleMotorFeedforward(kS.get(), kV.get()), kV, kS);

    inputs.shooterConnected = shooter1ConnectedDebouncer.calculate(!sparkUtil.sparkStickyFault);
    Measurement m1 = laserCan.getMeasurement();
    if (m1 == null) {
      inputs.laserCanDistanceM = Millimeters.of(-1);
    } else if (m1.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      inputs.laserCanDistanceM = Millimeters.of(m1.distance_mm);
    } else if (m1.status == LaserCan.LASERCAN_STATUS_WEAK_SIGNAL) {
      inputs.laserCanDistanceM = Millimeters.of(-1);
    }
  }

  @Override
  public void shooterOpenLoop(Voltage voltage) {
    shooterMotor.setVoltage(voltage);
  }

  @Override
  public void setShooterVelocity(AngularVelocity velocity) {
    shooterClosedLoopController.setSetpoint(
        velocity.in(RPM),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforward.calculate(velocity.in(RadiansPerSecond)));
  }

  @Override
  public void setShooterVelocityAndAcceleration(
      AngularVelocity velocity, AngularVelocity lastVelocity) {
    shooterClosedLoopController.setSetpoint(
        velocity.in(RPM),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforward.calculateWithVelocities(
            lastVelocity.in(RadiansPerSecond), velocity.in(RadiansPerSecond)));
  }

  public void follow(ShooterIOSpark leader, boolean inverted) {
    sparkUtil.tryUntilOk(
        shooterMotor,
        5,
        () ->
            shooterMotor.configure(
                getShooterConfig().follow(leader.shooterMotor, inverted),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }
}
