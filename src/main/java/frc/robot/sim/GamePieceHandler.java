package frc.robot.sim;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.sim.maplesim.BunnybotsLuniteOnFly;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.groundintake.GroundIntakeConstants.GroundIntakePivotConstants;
import frc.robot.subsystems.groundintake.GroundIntakeConstants.GroundIntakeRollerConstants.GroundIntakeRollerSimConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerSimConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ProjectileSpeedUtils;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

public class GamePieceHandler {
  private Supplier<Pose2d> drivePose;
  private GroundIntake groundIntake;
  private Indexer indexer;
  private Shooter shooter;
  private IntakeSimulation intakeSimulation;
  private ArrayList<Distance> gamePiecePositions;

  public GamePieceHandler(
      AbstractDriveTrainSimulation driveTrainSimulation,
      GroundIntake groundIntake,
      Indexer indexer,
      Shooter shooter) {
    this.drivePose = driveTrainSimulation::getSimulatedDriveTrainPose;
    this.groundIntake = groundIntake;
    this.indexer = indexer;
    this.shooter = shooter;
    intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "lunite",
            driveTrainSimulation,
            Inches.of(25),
            Inches.of(5.108870),
            IntakeSide.FRONT,
            1);

    intakeSimulation.setCustomIntakeCondition(
        (gamePiece) ->
            groundIntake.inputsPivot.groundIntakePivotPositionRads.isNear(
                GroundIntakePivotConstants.MIN_ANGLE,
                GroundIntakePivotConstants.PIVOT_POSITION_TOLERANCE));
    gamePiecePositions = new ArrayList<Distance>();
    new Trigger(
            () ->
                groundIntake.inputsRoller.groundIntakeRollerVelocityRadsPerSec.gt(
                    GroundIntakeRollerSimConstants.MIN_INTAKING_VELOCITY))
        .onTrue(Commands.runOnce(intakeSimulation::startIntake))
        .onFalse(Commands.runOnce(intakeSimulation::stopIntake));
    new Trigger(
            () ->
                indexer.inputs.indexerVelocityRadPerSec.gt(
                        IndexerSimConstants.MIN_INTAKING_VELOCITY)
                    && intakeSimulation.obtainGamePieceFromIntake())
        .onFalse(Commands.runOnce(() -> gamePiecePositions.add(Meters.of(0.0))));
  }

  public void periodic() {
    ArrayList<Pose3d> gamePiecePosesInIndexer = new ArrayList<Pose3d>();
    for (int i = gamePiecePositions.size() - 1; i >= 0; i--) {
      // Integrates the velocity of the indexer to update the position of the game piece in the
      // indexer
      gamePiecePositions.set(
          i,
          gamePiecePositions
              .get(i)
              .plus(
                  ProjectileSpeedUtils.calcProjectileSpeed(
                          indexer.inputs.indexerVelocityRadPerSec,
                          IndexerConstants.SPEED_TRANSFER_PERCENTAGE,
                          IndexerConstants.INDEXER_WHEEL_RADIUS)
                      .times(Milliseconds.of(20))));
      if (gamePiecePositions.get(i).lt(Meters.of(0))) {
        if (groundIntake.inputsPivot.groundIntakePivotPositionRads.isNear(
                GroundIntakePivotConstants.MIN_ANGLE,
                GroundIntakePivotConstants.PIVOT_POSITION_TOLERANCE)
            && groundIntake.inputsRoller.groundIntakeRollerVelocityRadsPerSec.lt(
                GroundIntakeRollerSimConstants.MIN_INTAKING_VELOCITY.unaryMinus())) {
          // TODO
        } else {
          gamePiecePositions.remove(i);
          SimulatedArena.getInstance()
              .addGamePieceProjectile(
                  new BunnybotsLuniteOnFly(
                      drivePose.get().getTranslation(), null, null, null, null, null, null));
        }
      }

      // interpolates pose of game piece through indexer
      if (gamePiecePositions.get(i).lt(IndexerSimConstants.LOWER_INDEXER_LENGTH)) {
        gamePiecePosesInIndexer.add(
            IndexerSimConstants.LUNITE_INITIAL_POSE.plus(
                IndexerSimConstants.LUNITE_MIDDLE_POSE
                    .minus(IndexerSimConstants.LUNITE_INITIAL_POSE)
                    .times(
                        gamePiecePositions.get(i).in(Meters)
                            / IndexerSimConstants.LOWER_INDEXER_LENGTH.in(Meters))));
      } else {
        gamePiecePosesInIndexer.add(
            IndexerSimConstants.LUNITE_MIDDLE_POSE.plus(
                IndexerSimConstants.LUNITE_FINAL_POSE
                    .minus(IndexerSimConstants.LUNITE_MIDDLE_POSE)
                    .times(
                        (gamePiecePositions.get(i).in(Meters)
                                - IndexerSimConstants.LOWER_INDEXER_LENGTH.in(Meters))
                            / (IndexerSimConstants.INDEXER_LENGTH.in(Meters)
                                - IndexerSimConstants.LOWER_INDEXER_LENGTH.in(Meters)))));
      }

      Logger.recordOutput(
          "GamePieceSimulation/IndexerGamePieces", (Pose3d[]) (gamePiecePosesInIndexer.toArray()));
    }
  }
}
