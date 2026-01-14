package frc.robot.sim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.sim.BunnybotsCosmicConverter.ConverterPosition;
import frc.robot.sim.BunnybotsStarSpire.SpirePosition;
import frc.robot.util.PoseUtils;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

public class BunnybotsLuniteOnFly extends GamePieceProjectile {

  public BunnybotsLuniteOnFly(
      Translation2d robotPosition,
      Translation2d shooterPositionOnRobot,
      ChassisSpeeds chassisSpeedsFieldRelative,
      Rotation2d shooterFacing,
      Distance initialHeight,
      LinearVelocity launchingSpeed,
      Angle shooterAngle) {
    super(
        BunnybotsLuniteOnField.BUNNYBOTS_LUNITE_INFO,
        robotPosition,
        shooterPositionOnRobot,
        chassisSpeedsFieldRelative,
        shooterFacing,
        initialHeight,
        launchingSpeed,
        shooterAngle);

    withTouchGroundHeight(0.6); // TODO: maybe lower?
    enableBecomesGamePieceOnFieldAfterTouchGround();
  }

  @Override
  public void addGamePieceAfterTouchGround(SimulatedArena arena) {
    if (!super.becomesGamePieceOnGroundAfterTouchGround) return;
    arena.addGamePiece(
        new GamePieceOnFieldSimulation(
            BunnybotsLuniteOnField.BUNNYBOTS_LUNITE_INFO,
            () ->
                Math.max(
                    BunnybotsLuniteOnField.BUNNYBOTS_LUNITE_INFO.gamePieceHeight().in(Meters) / 2,
                    getPositionAtTime(super.launchedTimer.get()).getZ()),
            new Pose2d(
                getPositionAtTime(launchedTimer.get()).toTranslation2d(),
                initialLaunchingVelocityMPS.getAngle()),
            super.initialLaunchingVelocityMPS));
  }

  public BunnybotsLuniteOnFly asSolarCoreShotLunite(
      Runnable hitTargetCallBack, ConverterPosition targetPosition) {
    return (BunnybotsLuniteOnFly)
        super.withTargetPosition(
                () ->
                    new Translation3d(PoseUtils.allianceFlip(targetPosition.pose.getTranslation()))
                        .plus(new Translation3d(0, 0, Units.inchesToMeters(70))))
            .withTargetTolerance(
                new Translation3d(
                        Inches.of(12 /* Adjust based on testing */), Inches.of(30), Inches.of(20))
                    .div(2))
            .withHitTargetCallBack(hitTargetCallBack);
  }

  public BunnybotsLuniteOnFly asShieldCoreShotLunite(
      Runnable hitTargetCallBack, ConverterPosition targetPosition) {
    return (BunnybotsLuniteOnFly)
        super.withTargetPosition(
                () ->
                    new Translation3d(PoseUtils.allianceFlip(targetPosition.pose.getTranslation()))
                        .plus(
                            new Translation3d(
                                0.485 - 0.191, 0, Units.inchesToMeters(51.961524 / 2))))
            .withTargetTolerance(
                new Translation3d(
                        Inches.of(30),
                        Inches.of(40),
                        Inches.of(
                            51.961524 /* Adjust based on testing, it may be too high and take shots from the core */))
                    .div(2))
            .withHitTargetCallBack(hitTargetCallBack);
  }

  public static void spawnLuniteAtSpire(SpirePosition position) {
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new BunnybotsLuniteOnFly(
                position.pose.getTranslation(),
                new Translation2d(0, 0),
                new ChassisSpeeds(),
                position.pose.getRotation(),
                Inches.of(24.448),
                MetersPerSecond.of(BunnybotsStarSpire.randomThrowSpeed()),
                Degrees.of(-26.249026)));
  }
}
