package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.PhoenixUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim extends GyroIO {
  private final GyroSimulation gyroSimulation;

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = gyroSimulation.getGyroReading();
    inputs.yawVelocityRadPerSec = gyroSimulation.getMeasuredAngularVelocity();

    inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
  }

  @Override
  public void resetRotation() {
    gyroSimulation.setRotation(new Rotation2d());
  }
}
