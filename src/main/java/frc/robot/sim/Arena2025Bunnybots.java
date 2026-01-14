package frc.robot.sim;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.sim.BunnybotsCosmicConverter.ShieldGenerator;
import frc.robot.sim.BunnybotsCosmicConverter.SolarCore;
import frc.robot.sim.BunnybotsStarSpire.HumanBehavior;
import frc.robot.sim.BunnybotsStarSpire.SpirePosition;
import frc.robot.util.PoseUtils;
import java.util.Arrays;
import org.ironmaple.simulation.SimulatedArena;

public class Arena2025Bunnybots extends SimulatedArena {
  public static final class BunnybotsFieldObstaclesMap extends FieldMap {
    public BunnybotsFieldObstaclesMap() {
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

      // Blue Cosmic Converters
      addRectangularObstacle(0.952500, 1.041400, new Pose2d(0.463, 5.069, new Rotation2d()));

      addRectangularObstacle(0.952500, 1.041400, new Pose2d(0.463, .557, new Rotation2d()));

      // Red Cosmic Converters
      addRectangularObstacle(
          0.952500, 1.041400, PoseUtils.flip(new Pose2d(0.463, 5.069, new Rotation2d())));

      addRectangularObstacle(
          0.952500, 1.041400, PoseUtils.flip(new Pose2d(0.463, .557, new Rotation2d())));

      // Red Star Spires
      addRectangularObstacle(
          0.190500, 0.61, new Pose2d(0.089 + 0.102 / 2, 6.139 + 1.067 / 2, new Rotation2d()));

      addRectangularObstacle(0.610, 0.102, new Pose2d(1.932, 8.204, new Rotation2d()));

      // Blue Star Spires
      addRectangularObstacle(
          0.190500,
          0.61,
          PoseUtils.flip(new Pose2d(0.089 + 0.102 / 2, 6.139 + 1.067 / 2, new Rotation2d())));
      addRectangularObstacle(
          0.610, 0.102, PoseUtils.flip(new Pose2d(1.932, 8.204, new Rotation2d())));

      // Middle Box
      addRectangularObstacle(
          0.869950,
          1.038225,
          new Pose2d(
              Constants.FIELD_WIDTH.in(Meters) / 2,
              Constants.FIELD_HEIGHT.in(Meters) / 2,
              new Rotation2d()));

      // Angled Boxes on Blue Side
      addRectangularObstacle(
          0.869950, 0.609600, new Pose2d(6.4645, 6.185, new Rotation2d(Math.PI / 4)));
      addRectangularObstacle(
          0.869950,
          0.609600,
          new Pose2d(
              PoseUtils.vertFlip(new Translation2d(6.4645, 6.185)), new Rotation2d(-Math.PI / 4)));

      // Angled Boxes on Red Side
      addRectangularObstacle(
          0.869950,
          0.609600,
          PoseUtils.flip(new Pose2d(6.4645, 6.185, new Rotation2d(Math.PI / 4))));
      addRectangularObstacle(
          0.869950,
          0.609600,
          PoseUtils.flip(
              new Pose2d(
                  PoseUtils.vertFlip(new Translation2d(6.4645, 6.185)),
                  new Rotation2d(-Math.PI / 4))));
    }
  }

  protected final SolarCore blueNearSolarCore;
  protected final SolarCore blueFarSolarCore;
  protected final SolarCore redNearSolarCore;
  protected final SolarCore redFarSolarCore;

  protected final ShieldGenerator blueNearShieldGenerator;
  protected final ShieldGenerator blueFarShieldGenerator;
  protected final ShieldGenerator redNearShieldGenerator;
  protected final ShieldGenerator redFarShieldGenerator;

  public Arena2025Bunnybots(HumanBehavior behavior) {
    super(new BunnybotsFieldObstaclesMap());
    addCustomSimulation(new BunnybotsStarSpire(this, Alliance.Blue, SpirePosition.FAR, behavior));
    addCustomSimulation(new BunnybotsStarSpire(this, Alliance.Red, SpirePosition.FAR, behavior));
    addCustomSimulation(new BunnybotsStarSpire(this, Alliance.Blue, SpirePosition.NEAR, behavior));
    addCustomSimulation(new BunnybotsStarSpire(this, Alliance.Red, SpirePosition.NEAR, behavior));

    blueNearSolarCore = new SolarCore(this, true, BunnybotsCosmicConverter.ConverterPosition.NEAR);
    super.addCustomSimulation(blueNearSolarCore);

    blueFarSolarCore = new SolarCore(this, true, BunnybotsCosmicConverter.ConverterPosition.FAR);
    super.addCustomSimulation(blueFarSolarCore);

    redNearSolarCore = new SolarCore(this, false, BunnybotsCosmicConverter.ConverterPosition.NEAR);
    super.addCustomSimulation(redNearSolarCore);

    redFarSolarCore = new SolarCore(this, false, BunnybotsCosmicConverter.ConverterPosition.FAR);
    super.addCustomSimulation(redFarSolarCore);

    blueNearShieldGenerator =
        new ShieldGenerator(this, true, BunnybotsCosmicConverter.ConverterPosition.NEAR);
    super.addCustomSimulation(blueNearShieldGenerator);

    blueFarShieldGenerator =
        new ShieldGenerator(this, true, BunnybotsCosmicConverter.ConverterPosition.FAR);
    super.addCustomSimulation(blueFarShieldGenerator);

    redNearShieldGenerator =
        new ShieldGenerator(this, false, BunnybotsCosmicConverter.ConverterPosition.NEAR);
    super.addCustomSimulation(redNearShieldGenerator);

    redFarShieldGenerator =
        new ShieldGenerator(this, false, BunnybotsCosmicConverter.ConverterPosition.FAR);
    super.addCustomSimulation(redFarShieldGenerator);
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
