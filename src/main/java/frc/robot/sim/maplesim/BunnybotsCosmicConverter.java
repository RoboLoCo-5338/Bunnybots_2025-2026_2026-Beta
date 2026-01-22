package frc.robot.sim.maplesim;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.PoseUtils;
import java.util.List;
import org.ironmaple.simulation.Goal;
import org.ironmaple.simulation.gamepieces.GamePiece;

public class BunnybotsCosmicConverter {
  public static class SolarCore extends Goal {
    private final Arena2025Bunnybots arena;

    public SolarCore(Arena2025Bunnybots arena, boolean isBlue, ConverterPosition position) {
      super(
          arena,
          Inches.of(12 /* Adjust based on testing */),
          Inches.of(30),
          Inches.of(20),
          "Lunite",
          (new Translation3d(
                  isBlue
                      ? position.pose.getTranslation()
                      : PoseUtils.flip(position.pose.getTranslation()))
              .plus(new Translation3d(0, 0, Units.inchesToMeters(60)))),
          isBlue);
      this.arena = arena;
    }

    @Override
    protected void addPoints() {
      arena.addValueToMatchBreakdown(isBlue, "TotalLunitesScoredOn SolarCore", 1);
      arena.addValueToMatchBreakdown(
          isBlue, "Auto/LunitesScoredInAutoOn SolarCore", DriverStation.isAutonomous() ? 1 : 0);
      arena.addToScore(isBlue, DriverStation.isAutonomous() ? 7 : 5);
    }

    @Override
    public void draw(List<Pose3d> drawList) {
      return;
    }
  }

  public static class ShieldGenerator extends Goal {
    private final Arena2025Bunnybots arena;

    public ShieldGenerator(Arena2025Bunnybots arena, boolean isBlue, ConverterPosition position) {
      super(
          arena,
          Inches.of(30),
          Inches.of(40),
          Inches.of(
              51.961524 /* Adjust based on testing, it may be too high and take shots from the core */),
          "Lunite",
          (new Translation3d(
                  isBlue
                      ? position.pose.getTranslation()
                      : PoseUtils.flip(position.pose.getTranslation()))
              .plus(new Translation3d(0.485 - 0.191, 0, 0))),
          isBlue);
      this.arena = arena;
    }

    @Override
    protected void addPoints() {
      arena.addValueToMatchBreakdown(isBlue, "TotalLunitesScoredOn ShieldGenerator", 1);
      arena.addValueToMatchBreakdown(
          isBlue,
          "Auto/LunitesScoredInAutoOn ShieldGenerator",
          DriverStation.isAutonomous() ? 1 : 0);
      arena.addToScore(isBlue, DriverStation.isAutonomous() ? 4 : 2);
    }

    @Override
    public void draw(List<Pose3d> drawList) {
      return;
    }

    @Override
    protected boolean checkVel(GamePiece gamePiece) {
      return gamePiece.getVelocity3dMPS().getZ() < 0
          || gamePiece.getPose3d().getY() > Units.inchesToMeters(22);
    }
  }

  public static enum ConverterPosition {
    NEAR(new Pose2d(0.191, 5.069, Rotation2d.fromDegrees(0))),
    FAR(new Pose2d(0.191, 5.069, Rotation2d.fromDegrees(0)));

    public final Pose2d pose;

    ConverterPosition(Pose2d pose) {
      this.pose = pose;
    }
  }
}
