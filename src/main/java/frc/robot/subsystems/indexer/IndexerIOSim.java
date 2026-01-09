package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerSimConstants;
import org.littletonrobotics.junction.Logger;

public class IndexerIOSim extends IndexerIOTalonFX {
  TalonFXSimState simMotor = indexerMotor.getSimState();
  FlywheelSim physicsSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1),
              IndexerConstants.MOI.in(KilogramSquareMeters),
              IndexerConstants.IndexerSimConstants.GEARING),
          DCMotor.getKrakenX60(1));

  public IndexerIOSim() {
    super();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
    physicsSim.setInputVoltage(simMotor.getMotorVoltage());

    Logger.recordOutput("Indexer/indexerVelocity", physicsSim.getAngularVelocityRPM());
    Logger.recordOutput("Indexer/indexerAppliedVolts", physicsSim.getInputVoltage());
    Logger.recordOutput("Indexer/indexerCurrentAmps", physicsSim.getCurrentDrawAmps());

    physicsSim.update(0.02);

    simMotor.addRotorPosition(
        Units.radiansToRotations(physicsSim.getAngularVelocityRadPerSec())
            * 0.02
            * IndexerSimConstants.GEARING);
    simMotor.setRotorVelocity(
        Units.radiansToRotations(physicsSim.getAngularVelocityRadPerSec())
            * IndexerSimConstants.GEARING);

    super.updateInputs(inputs);
  }
}
