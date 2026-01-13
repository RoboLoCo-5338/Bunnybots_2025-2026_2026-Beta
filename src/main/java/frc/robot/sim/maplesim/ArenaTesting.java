package frc.robot.sim.maplesim;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.sim.maplesim.BunnybotsStarSpire.HumanBehavior;
import frc.robot.util.PoseUtils;
import java.util.Arrays;
import org.ironmaple.simulation.SimulatedArena;

public class ArenaTesting extends SimulatedArena {
  public static final class TestFieldObstaclesMap extends FieldMap {
    public TestFieldObstaclesMap() {
      super();

      // Blue alliance wall
      addBorderLine(new Translation2d(), new Translation2d(0.117 * 2, 8.28));

      // Red alliance wall
      addBorderLine(
          PoseUtils.flip(new Translation2d()), PoseUtils.flip(new Translation2d(0.117 * 2, 8.2)));

      // Lower Wall
      addBorderLine(new Translation2d(0, 0.075), PoseUtils.flip(new Translation2d()));

      // Upper Wall
      addBorderLine(new Translation2d(0, 8.2), PoseUtils.flip(new Translation2d(0, 8.2)));

      // That blue box thingy in front of the blue alliance wall
      Translation2d[] blueBoxVortices =
          new Translation2d[] {
            new Translation2d(0.117, 6.139),
            new Translation2d(2.863, 6.139),
            new Translation2d(2.863, 2.868 - 0.26),
            new Translation2d(2.354, 2.868 - 0.26),
            new Translation2d(2.354, 5.649),
            new Translation2d(0.117, 5.649)
          };

      for (int i = 0; i < 6; i++)
        super.addBorderLine(blueBoxVortices[i], blueBoxVortices[(i + 1) % 6]);

      // That red box thingy in front of the blue alliance wall
      Translation2d[] redBoxVortices =
          Arrays.stream(blueBoxVortices)
              .map(
                  pointAtBlue ->
                      new Translation2d(
                          Constants.FIELD_WIDTH.in(Meters) - pointAtBlue.getX(),
                          pointAtBlue.getY()))
              .toArray(Translation2d[]::new);
      for (int i = 0; i < 6; i++)
        super.addBorderLine(redBoxVortices[i], redBoxVortices[(i + 1) % 6]);
    }
  }

  public ArenaTesting(HumanBehavior behavior) {
    super(new TestFieldObstaclesMap());
  }

  @Override
  public void placeGamePiecesOnField() {
    Translation2d[] bluePositions =
        new Translation2d[] {
          new Translation2d(
              2.863 - ((0.31 + 0.02) / 2) + 0.13, 2.868 - 0.26 - ((0.671 + 0.84) / 2) + 0.125),
          new Translation2d(
              2.863 - ((0.31 + 0.02) / 2) + 0.13,
              2.868 - 0.26 - ((0.671 + 0.84) / 2) - ((0.779 + 0.440) / 2) + 0.125),
          new Translation2d(
              2.863 - ((0.31 + 0.02) / 2) + 0.13,
              2.868 - 0.26 - ((0.671 + 0.84) / 2) - ((1.05 + 1.388) / 2) + 0.125)
        };

    for (Translation2d position : bluePositions) addGamePiece(new BunnybotsLuniteOnField(position));

    Translation2d[] redPositions =
        Arrays.stream(bluePositions)
            .map(
                bluePosition ->
                    new Translation2d(
                        Constants.FIELD_WIDTH.in(Meters) - bluePosition.getX(),
                        bluePosition.getY()))
            .toArray(Translation2d[]::new);

    for (Translation2d position : redPositions) addGamePiece(new BunnybotsLuniteOnField(position));

    // TODO: setup value for lunite stuff
    setupValueForMatchBreakdown("TotalLunitesScoredOn SolarCore");
    setupValueForMatchBreakdown("TotalLunitesScoredOn ShieldGenerator");
    setupValueForMatchBreakdown("Auto/LunitesScoredInAutoOn SolarCore");
    setupValueForMatchBreakdown("Auto/LunitesScoredInAutoOn ShieldGenerator");
  }
}
