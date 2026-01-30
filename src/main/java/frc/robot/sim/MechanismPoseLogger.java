package frc.robot.sim;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.groundintake.GroundIntakeConstants.GroundIntakePivotConstants.GroundIntakePivotSimConstants;
import frc.robot.subsystems.groundintake.GroundIntakeConstants.GroundIntakeRollerConstants.GroundIntakeRollerSimConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerSimConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterSimConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class MechanismPoseLogger {
  private GroundIntake groundIntake;
  private Indexer indexer;
  private Shooter shooter;

  public MechanismPoseLogger(GroundIntake groundIntake, Indexer indexer, Shooter shooter) {
    this.groundIntake = groundIntake;
    this.indexer = indexer;
    this.shooter = shooter;

    Logger.recordOutput("Origin", new Pose3d());
  }

  @AutoLogOutput(key = "MechanismPoses/GroundIntakePose")
  protected Pose3d getGroundIntakePose() {
    return new Pose3d(
        GroundIntakePivotSimConstants.GROUNDINTAKE_ORIGIN,
        new Rotation3d(0, -groundIntake.inputsPivot.groundIntakePivotPositionRads.in(Radians), 0));
  }

  @AutoLogOutput(key = "MechanismPoses/GroundIntakeShortWheelPose")
  protected Pose3d getGroundIntakeShortWheelPose() {
    Translation2d verticalGroundIntakePivotTranslation =
        new Translation2d(
            GroundIntakePivotSimConstants.GROUNDINTAKE_ORIGIN.getX(),
            GroundIntakePivotSimConstants.GROUNDINTAKE_ORIGIN.getZ());
    Translation2d verticalGroundIntakeFrontWheelTranslation =
        new Translation2d(
            GroundIntakeRollerSimConstants.GROUNDINTAKE_SHORT_WHEEL_ORIGIN.getX(),
            GroundIntakeRollerSimConstants.GROUNDINTAKE_SHORT_WHEEL_ORIGIN.getZ());
    verticalGroundIntakeFrontWheelTranslation =
        verticalGroundIntakeFrontWheelTranslation.rotateAround(
            verticalGroundIntakePivotTranslation,
            new Rotation2d(groundIntake.inputsPivot.groundIntakePivotPositionRads.in(Radians)));
    return new Pose3d(
        verticalGroundIntakeFrontWheelTranslation.getX(),
        GroundIntakeRollerSimConstants.GROUNDINTAKE_SHORT_WHEEL_ORIGIN.getY(),
        verticalGroundIntakeFrontWheelTranslation.getY(),
        new Rotation3d(
            0, -groundIntake.inputsRoller.groundIntakeRollerPositionRads.in(Radians), 0));
  }

  @AutoLogOutput(key = "MechanismPoses/GroundIntakeBackLongWheelPose")
  protected Pose3d getGroundIntakeBackLongWheelPose() {
    Translation2d verticalGroundIntakePivotTranslation =
        new Translation2d(
            GroundIntakePivotSimConstants.GROUNDINTAKE_ORIGIN.getX(),
            GroundIntakePivotSimConstants.GROUNDINTAKE_ORIGIN.getZ());
    Translation2d verticalGroundIntakeBackLongWheelTranslation =
        new Translation2d(
            GroundIntakeRollerSimConstants.GROUNDINTAKE_BACK_LONG_WHEEL_ORIGIN.getX(),
            GroundIntakeRollerSimConstants.GROUNDINTAKE_BACK_LONG_WHEEL_ORIGIN.getZ());
    verticalGroundIntakeBackLongWheelTranslation =
        verticalGroundIntakeBackLongWheelTranslation.rotateAround(
            verticalGroundIntakePivotTranslation,
            new Rotation2d(groundIntake.inputsPivot.groundIntakePivotPositionRads.in(Radians)));
    return new Pose3d(
        verticalGroundIntakeBackLongWheelTranslation.getX(),
        GroundIntakeRollerSimConstants.GROUNDINTAKE_SHORT_WHEEL_ORIGIN.getY(),
        verticalGroundIntakeBackLongWheelTranslation.getY(),
        new Rotation3d(
            0, -groundIntake.inputsRoller.groundIntakeRollerPositionRads.in(Radians), 0));
  }

  @AutoLogOutput(key = "MechanismPoses/GroundIntakeFrontLongWheelPose")
  protected Pose3d getGroundIntakeFrontLongWheelPose() {
    Translation2d verticalGroundIntakePivotTranslation =
        new Translation2d(
            GroundIntakePivotSimConstants.GROUNDINTAKE_ORIGIN.getX(),
            GroundIntakePivotSimConstants.GROUNDINTAKE_ORIGIN.getZ());
    Translation2d verticalGroundIntakeFrontLongWheelTranslation =
        new Translation2d(
            GroundIntakeRollerSimConstants.GROUNDINTAKE_FRONT_LONG_WHEEL_ORIGIN.getX(),
            GroundIntakeRollerSimConstants.GROUNDINTAKE_FRONT_LONG_WHEEL_ORIGIN.getZ());
    verticalGroundIntakeFrontLongWheelTranslation =
        verticalGroundIntakeFrontLongWheelTranslation.rotateAround(
            verticalGroundIntakePivotTranslation,
            new Rotation2d(groundIntake.inputsPivot.groundIntakePivotPositionRads.in(Radians)));
    return new Pose3d(
        verticalGroundIntakeFrontLongWheelTranslation.getX(),
        GroundIntakeRollerSimConstants.GROUNDINTAKE_SHORT_WHEEL_ORIGIN.getY(),
        verticalGroundIntakeFrontLongWheelTranslation.getY(),
        new Rotation3d(
            0, -groundIntake.inputsRoller.groundIntakeRollerPositionRads.in(Radians), 0));
  }

  // TODO: check which shooter is top and bottom
  @AutoLogOutput(key = "MechanismPoses/BottomShooterWheelPose")
  protected Pose3d getBottomShooterWheelPose() {
    return new Pose3d(
        ShooterSimConstants.BOTTOM_SHOOTER_ORIGIN,
        new Rotation3d(0, -shooter.inputs1.shooterPositionRads.in(Radians), 0));
  }

  @AutoLogOutput(key = "MechanismPoses/TopShooterWheelPose")
  protected Pose3d getTopShooterWheelPose() {
    return new Pose3d(
        ShooterSimConstants.TOP_SHOOTER_ORIGIN,
        new Rotation3d(0, -shooter.inputs2.shooterPositionRads.in(Radians), 0));
  }

  @AutoLogOutput(key = "MechanismPoses/FrontIndexerWheelPose")
  protected Pose3d getFrontIndexerWheelPose() {
    return new Pose3d(
        IndexerSimConstants.FRONT_INDEXER_WHEEL_ORIGIN,
        new Rotation3d(0, -indexer.inputs.indexerPositionRads.in(Radians), 0));
  }

  @AutoLogOutput(key = "MechanismPoses/FrontMiddleIndexerWheelPose")
  protected Pose3d getFrontMiddleIndexerWheelPose() {
    return new Pose3d(
        IndexerSimConstants.FRONT_MIDDLE_INDEXER_WHEEL_ORIGIN,
        new Rotation3d(0, -indexer.inputs.indexerPositionRads.in(Radians), 0));
  }

  @AutoLogOutput(key = "MechanismPoses/BackMiddleIndexerWheelPose")
  protected Pose3d getBackMiddleIndexerWheelPose() {
    return new Pose3d(
        IndexerSimConstants.BACK_MIDDLE_INDEXER_WHEEL_ORIGIN,
        new Rotation3d(0, -indexer.inputs.indexerPositionRads.in(Radians), 0));
  }

  @AutoLogOutput(key = "MechanismPoses/BackIndexerWheelPose")
  protected Pose3d getBackIndexerWheelPose() {
    return new Pose3d(
        IndexerSimConstants.BACK_INDEXER_WHEEL_ORIGIN,
        new Rotation3d(0, -indexer.inputs.indexerPositionRads.in(Radians), 0));
  }

  @AutoLogOutput(key = "MechanismPoses/GearedIndexerWheelPose")
  protected Pose3d getGearedIndexerWheelPose() {
    return new Pose3d(
        IndexerSimConstants.GEARED_INDEXER_WHEEL_ORIGIN,
        new Rotation3d(0, indexer.inputs.indexerPositionRads.in(Radians), 0));
  }
}
