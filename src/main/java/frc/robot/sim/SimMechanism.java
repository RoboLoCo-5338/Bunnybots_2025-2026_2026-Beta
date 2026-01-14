package frc.robot.sim;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.ArrayList;

public interface SimMechanism {
  static final ArrayList<SimMechanism> MECHANISMS = new ArrayList<SimMechanism>();

  public default void initSimVoltage() {
    MECHANISMS.add(this);
  }

  public static void updateBatteryVoltages() {
    ArrayList<Double> currents = new ArrayList<Double>();
    for (SimMechanism m : MECHANISMS) {
      for (double current : m.getCurrents()) {
        currents.add(current);
      }
    }
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            currents.stream().mapToDouble(Double::doubleValue).toArray()));
  }

  public abstract double[] getCurrents();
}
