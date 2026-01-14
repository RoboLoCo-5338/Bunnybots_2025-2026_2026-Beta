// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.sim.Arena2025Bunnybots;
import frc.robot.sim.BunnybotsStarSpire.HumanBehavior;
import frc.robot.sim.SimMechanism;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.groundintake.GroundIntakeConstants;
import frc.robot.subsystems.groundintake.GroundIntakePivotIOSim;
import frc.robot.subsystems.groundintake.GroundIntakePivotIOTalonFX;
import frc.robot.subsystems.groundintake.GroundIntakeRollerIOSim;
import frc.robot.subsystems.groundintake.GroundIntakeRollerIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private SwerveDriveSimulation driveSimulation = null;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final GroundIntake groundIntake;
  private final Indexer indexer;
  private final Shooter shooter;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft, Drive.TUNABLE_PID_VALUES),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight, Drive.TUNABLE_PID_VALUES),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft, Drive.TUNABLE_PID_VALUES),
                new ModuleIOTalonFXReal(TunerConstants.BackRight, Drive.TUNABLE_PID_VALUES),
                (pose) -> {});
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0));
        groundIntake =
            new GroundIntake(new GroundIntakeRollerIOTalonFX(), new GroundIntakePivotIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        shooter = new Shooter(new ShooterIOSpark(1), new ShooterIOSpark(2));
        break;

      case SIM:
        driveSimulation =
            new SwerveDriveSimulation(
                Drive.mapleSimConfig, new Pose2d(3, 0.5, new Rotation2d(Math.PI)));
        SimulatedArena.overrideInstance(new Arena2025Bunnybots(HumanBehavior.ON));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(
                    TunerConstants.FrontLeft,
                    Drive.TUNABLE_PID_VALUES,
                    driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(
                    TunerConstants.FrontRight,
                    Drive.TUNABLE_PID_VALUES,
                    driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(
                    TunerConstants.BackLeft,
                    Drive.TUNABLE_PID_VALUES,
                    driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(
                    TunerConstants.BackRight,
                    Drive.TUNABLE_PID_VALUES,
                    driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name,
                    VisionConstants.robotToCamera0,
                    driveSimulation::getSimulatedDriveTrainPose));
        groundIntake =
            new GroundIntake(new GroundIntakeRollerIOSim(), new GroundIntakePivotIOSim());
        indexer = new Indexer(new IndexerIOSim());
        shooter = new Shooter(new ShooterIOSim(1), new ShooterIOSim(2));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO(),
                new ModuleIO(),
                new ModuleIO(),
                new ModuleIO(),
                new ModuleIO(),
                (pose) -> {});
        vision = new Vision(drive, new VisionIO() {});
        groundIntake =
            new GroundIntake(new GroundIntakeRollerIOTalonFX(), new GroundIntakePivotIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        shooter = new Shooter(new ShooterIO(), new ShooterIO());
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));

    drive.addRoutinesToChooser(autoChooser);
    indexer.addRoutinesToChooser(autoChooser);
    shooter.addRoutinesToChooser(autoChooser);
    groundIntake.addRoutinesToChooser(autoChooser);
  }

  public void manualButtonBindings() {
    // drive controls
    DriveCommands.joystickDrive(
        drive,
        () ->
            -driverController.getLeftY() * Math.pow(Math.abs(driverController.getLeftY()), 1.2 - 1),
        () ->
            -driverController.getLeftX() * Math.pow(Math.abs(driverController.getLeftX()), 1.2 - 1),
        () ->
            -driverController.getRightX()
                * Math.pow(Math.abs(driverController.getRightX()), 2.2 - 1));

    // indexer controls
    operatorController
        .rightBumper()
        .whileTrue(
            indexer.setIndexerVelocity(
                () -> IndexerConstants.INDEXER_INTAKE_VELOCITY)) // TODO: update value later
        .onFalse(indexer.setIndexerVelocity(() -> IndexerConstants.INDEXER_NO_VELOCITY));
    operatorController
        .leftBumper()
        .whileTrue(
            indexer.setIndexerVelocity(
                () -> IndexerConstants.INDEXER_OUTTAKE_VELOCITY)) // TODO: update value later
        .onFalse(indexer.setIndexerVelocity(() -> IndexerConstants.INDEXER_NO_VELOCITY));

    // shooter controls
    operatorController
        .leftTrigger()
        .whileTrue(
            shooter.setShooterVelocity(
                () -> ShooterConstants.SHOOTER_REVERSE_VELOCITY)) // TODO: update value later
        .onFalse(shooter.setShooterVelocity(() -> ShooterConstants.SHOOTER_NO_VELOCITY));
    operatorController
        .rightTrigger()
        .whileTrue(
            shooter.setShooterVelocity(
                () ->
                    ShooterConstants
                        .SHOOTER_HIGH_GOAL_VELOCITY)) // TODO: update value later to shoot in
        // high goal
        .onFalse(shooter.setShooterVelocity(() -> ShooterConstants.SHOOTER_NO_VELOCITY));
    operatorController
        .y()
        .onTrue(
            shooter.setShooterVelocity(
                () ->
                    ShooterConstants
                        .SHOOTER_LOW_GOAL_VELOCITY)) // TODO: update value later to shoot in
        // low goal
        .onFalse(shooter.setShooterVelocity(() -> ShooterConstants.SHOOTER_NO_VELOCITY));

    // ground intake controls
    groundIntake.setDefaultCommand(
        groundIntake.setGroundIntakeRollerVelocity(
            () ->
                RotationsPerSecond.of(
                    operatorController.getLeftY()
                        * GroundIntakeConstants.GroundIntakeRollerConstants
                            .GROUNDINTAKE_ROLLER_VELOCITY_MULTIPLIER))); // TODO: update value later
    groundIntake.setDefaultCommand(
        groundIntake.setGroundIntakePivotVelocity(
            () ->
                RotationsPerSecond.of(
                    operatorController.getRightY()
                        * GroundIntakeConstants.GroundIntakePivotConstants
                            .GROUND_INTAKE_PIVOT_VELOCITY_MULTIPLIER))); // TODO: update value later
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.CURRENT_MODE != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 0.5, new Rotation2d(Math.PI)));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.CURRENT_MODE != Constants.Mode.SIM) return;
    SimMechanism.updateBatteryVoltages();

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Lunites", SimulatedArena.getInstance().getGamePiecesArrayByType("Lunite"));
  }
}
