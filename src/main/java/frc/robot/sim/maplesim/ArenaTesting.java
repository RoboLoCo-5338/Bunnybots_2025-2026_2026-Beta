package frc.robot.sim.maplesim;

import frc.robot.sim.maplesim.BunnybotsStarSpire.HumanBehavior;
import org.ironmaple.simulation.SimulatedArena;

public class ArenaTesting extends SimulatedArena {
  public static final class TestFieldObstaclesMap extends FieldMap {
    public TestFieldObstaclesMap() {
      super();
    }
  }

  public ArenaTesting(HumanBehavior behavior) {
    super(new TestFieldObstaclesMap());
  }

  @Override
  public void placeGamePiecesOnField() {}
}
