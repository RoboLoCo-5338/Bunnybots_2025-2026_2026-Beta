package frc.robot.subsystems.groundintake;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.sim.SimMechanism;
import frc.robot.subsystems.groundintake.GroundIntakeConstants.GroundIntakeRollerConstants;
import frc.robot.subsystems.groundintake.GroundIntakeConstants.GroundIntakeRollerConstants.GroundIntakeRollerSimConstants;
import org.littletonrobotics.junction.Logger;

public class GroundIntakeRollerIOSim extends GroundIntakeRollerIOTalonFX implements SimMechanism {
  TalonFXSimState simMotor = groundIntakeRollerMotor.getSimState();
  FlywheelSim physicsSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1),
              GroundIntakeRollerSimConstants.MOI.in(KilogramSquareMeters),
              GroundIntakeRollerConstants.GEARING),
          DCMotor.getKrakenX60(1));

  public GroundIntakeRollerIOSim() {
    super();
    initSimVoltage();
  }

  @Override
  public void updateInputs(GroundIntakeRollerIOInputs inputs) {
    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
    physicsSim.setInputVoltage(simMotor.getMotorVoltage());

    Logger.recordOutput(
        "GroundIntakeRoller/GroundIntakeRollerVelocity", physicsSim.getAngularVelocityRPM());
    Logger.recordOutput(
        "GroundIntakeRoller/groundIntakeRollerAppliedVoltage", physicsSim.getInputVoltage());
    Logger.recordOutput(
        "GroundIntakeRoller/groundIntakeRollerCurrent", physicsSim.getCurrentDrawAmps());

    physicsSim.update(0.02);

    simMotor.addRotorPosition(
        Units.radiansToRotations(physicsSim.getAngularVelocityRadPerSec())
            * 0.02
            * GroundIntakeRollerConstants.GEARING);
    simMotor.setRotorVelocity(
        Units.radiansToRotations(physicsSim.getAngularVelocityRadPerSec())
            * GroundIntakeRollerConstants.GEARING);

    super.updateInputs(inputs);
  }

  @Override
  public double[] getCurrents() {
    return new double[] {physicsSim.getCurrentDrawAmps()};
  }
}
