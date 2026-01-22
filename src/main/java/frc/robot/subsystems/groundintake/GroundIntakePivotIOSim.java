package frc.robot.subsystems.groundintake;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.sim.SimMechanism;
import frc.robot.subsystems.groundintake.GroundIntakeConstants.GroundIntakePivotConstants;
import frc.robot.subsystems.groundintake.GroundIntakeConstants.GroundIntakePivotConstants.GroundIntakePivotSimConstants;
import org.littletonrobotics.junction.Logger;

public class GroundIntakePivotIOSim extends GroundIntakePivotIOTalonFX implements SimMechanism {
  TalonFXSimState simMotor = groundIntakePivotMotor.getSimState();
  DutyCycleEncoderSim simEncoder = new DutyCycleEncoderSim(groundIntakePivotEncoder);
  SingleJointedArmSim physicsSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          GroundIntakePivotConstants.MOTOR_TO_SENSOR_GEARING
              * GroundIntakePivotConstants.SENSOR_TO_PIVOT_GEARING,
          GroundIntakePivotConstants.MOI.in(KilogramSquareMeters),
          GroundIntakePivotSimConstants.LENGTH.in(Meters),
          GroundIntakePivotConstants.MIN_ANGLE.in(Radians),
          GroundIntakePivotConstants.MAX_ANGLE.in(Radians),
          true,
          GroundIntakePivotSimConstants.STARTING_ANGLE.in(Radians));

  public GroundIntakePivotIOSim() {
    super();
    initSimVoltage();
  }

  @Override
  public void updateInputs(GroundIntakePivotIOInputs inputs) {
    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
    physicsSim.setInputVoltage(simMotor.getMotorVoltage());

    Logger.recordOutput(
        "GroundIntakePivot/GroundIntakePivotPositionRads", physicsSim.getAngleRads());
    Logger.recordOutput(
        "GroundIntakePivot/GroundIntakePivotVelocityRadsPerSec", physicsSim.getVelocityRadPerSec());
    Logger.recordOutput("GroundIntakePivot/GroundIntakePivotAppliedVolts", physicsSim.getInput(0));
    Logger.recordOutput(
        "GroundIntakePivot/GroundIntakePivotCurrentAmps", physicsSim.getCurrentDrawAmps());

    physicsSim.update(0.02);

    simMotor.setRawRotorPosition(
        Units.radiansToRotations(physicsSim.getAngleRads())
            * GroundIntakePivotConstants.MOTOR_TO_SENSOR_GEARING
            * GroundIntakePivotConstants.SENSOR_TO_PIVOT_GEARING);
    simEncoder.set(
        Units.radiansToRotations(physicsSim.getAngleRads())
            * GroundIntakePivotConstants.SENSOR_TO_PIVOT_GEARING);
    simMotor.setRotorVelocity(
        Units.radiansToRotations(physicsSim.getVelocityRadPerSec())
            * GroundIntakePivotConstants.MOTOR_TO_SENSOR_GEARING
            * GroundIntakePivotConstants.SENSOR_TO_PIVOT_GEARING);

    super.updateInputs(inputs);
  }

  @Override
  public double[] getCurrents() {
    return new double[] {physicsSim.getCurrentDrawAmps()};
  }
}
