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

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.sim.MechanismPoseLogger;
import frc.robot.sim.SimMechanism;
import frc.robot.sim.maplesim.ArenaTesting;
import frc.robot.sim.maplesim.BunnybotsStarSpire.HumanBehavior;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.groundintake.GroundIntakeConstants;
import frc.robot.subsystems.groundintake.GroundIntakeConstants.GroundIntakePivotConstants;
import frc.robot.subsystems.groundintake.GroundIntakeConstants.GroundIntakeRollerConstants;
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

  private final MechanismPoseLogger mechanismPoseLogger;

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
                drive::accept,
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
        SimulatedArena.overrideInstance(new ArenaTesting(HumanBehavior.ON));
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
                drive::accept,
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
        vision = new Vision(drive::accept, new VisionIO() {});
        groundIntake =
            new GroundIntake(new GroundIntakeRollerIOTalonFX(), new GroundIntakePivotIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        shooter = new Shooter(new ShooterIO(), new ShooterIO());
        break;
    }

    RobotState.startInstance(drive, groundIntake, shooter, indexer, vision);

    // fix hte gi position stuff cuz iirc thats all worng
    // presets
    // start using gi
    NamedCommands.registerCommand(
        "Shoot High",
        new SequentialCommandGroup(
            shooter.setShooterVelocity(
                () -> ShooterConstants.SHOOTER_HIGH_GOAL_VELOCITY,
                () -> ShooterConstants.SHOOTER_HIGH_GOAL_VELOCITY.unaryMinus()),
            new WaitCommand(1.0),
            indexer.setIndexerVelocity(() -> RotationsPerSecond.of(3)),
            groundIntake.setGroundIntakeRollerVelocity(() -> RotationsPerSecond.of(3))));
    NamedCommands.registerCommand(
        "Shoot Low",
        new SequentialCommandGroup(
            shooter.setShooterVelocity(
                () -> ShooterConstants.SHOOTER_LOW_GOAL_VELOCITY,
                () -> ShooterConstants.SHOOTER_LOW_GOAL_VELOCITY.unaryMinus()),
            new WaitCommand(1.0),
            indexer.setIndexerVelocity(() -> RotationsPerSecond.of(3)),
            groundIntake.setGroundIntakeRollerVelocity(() -> RotationsPerSecond.of(3))));
    NamedCommands.registerCommand(
        "Indexer Intake", indexer.setIndexerVelocity(() -> RotationsPerSecond.of(3)));
    NamedCommands.registerCommand(
        "Stop Motors",
        new ParallelCommandGroup(
            shooter.setShooterVelocity(() -> RotationsPerSecond.of(0)),
            indexer.setIndexerVelocity(() -> RotationsPerSecond.of(0)),
            groundIntake.setGroundIntakeRollerVelocity(() -> RotationsPerSecond.of(0))));
    NamedCommands.registerCommand(
        "Lower GI",
        new SequentialCommandGroup(
            groundIntake.setGroundIntakePivotPosition(
                GroundIntakePivotConstants.MIN_ANGLE), // get the
            // correct
            // value
            groundIntake.setGroundIntakeRollerVelocity(() -> RPM.of(1200))));
    groundIntake
        .setGroundIntakePivotVoltage(
            () -> GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_VOLTAGE_DOWN)
        .repeatedly()
        .until(
            () -> {
              return groundIntake.inputsPivot.groundIntakePivotPositionRads.isNear(
                  GroundIntakePivotConstants.MIN_ANGLE,
                  GroundIntakeConstants.GroundIntakePivotConstants.PIVOT_POSITION_TOLERANCE);
            });
    NamedCommands.registerCommand(
        "Index",
        new SequentialCommandGroup(
            new WaitUntilCommand(
                () -> {
                  return indexer.inputs.indexerDistanceM.lt(Inches.of(3));
                }),
            indexer.setIndexerVelocity(() -> IndexerConstants.INDEXER_INTAKE_VELOCITY),
            new WaitUntilCommand(
                () -> {
                  return indexer.inputs.indexerDistanceM.gt(Inches.of(3));
                }),
            indexer.setIndexerVelocity(() -> RPM.of(0))));
    NamedCommands.registerCommand(
        "Raise GI",
        new SequentialCommandGroup(
            groundIntake
                .setGroundIntakePivotVoltage(
                    () -> GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_VOLTAGE_UP)
                .repeatedly()
                .until(
                    () -> {
                      return groundIntake.inputsPivot.groundIntakePivotPositionRads.isNear(
                          GroundIntakePivotConstants.MAX_ANGLE,
                          GroundIntakeConstants.GroundIntakePivotConstants
                              .PIVOT_POSITION_TOLERANCE);
                    })));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "No Leave High Outer 3",
        new SequentialCommandGroup(
            shooter.setShooterVelocity(
                () -> ShooterConstants.SHOOTER_HIGH_GOAL_VELOCITY,
                () -> ShooterConstants.SHOOTER_HIGH_GOAL_VELOCITY.unaryMinus()),
            new WaitCommand(1.0),
            indexer.setIndexerVelocity(() -> RotationsPerSecond.of(3)),
            groundIntake.setGroundIntakeRollerVelocity(() -> RotationsPerSecond.of(3))));
    autoChooser.addOption(
        "No Leave Low Outer 3",
        new SequentialCommandGroup(
            shooter.setShooterVelocity(
                () -> ShooterConstants.SHOOTER_LOW_GOAL_VELOCITY,
                () -> ShooterConstants.SHOOTER_LOW_GOAL_VELOCITY.unaryMinus()),
            new WaitCommand(1.0),
            indexer.setIndexerVelocity(() -> RotationsPerSecond.of(3)),
            groundIntake.setGroundIntakeRollerVelocity(() -> RotationsPerSecond.of(3))));
    drive.addRoutinesToChooser(autoChooser);
    indexer.addRoutinesToChooser(autoChooser);
    shooter.addRoutinesToChooser(autoChooser);
    groundIntake.addRoutinesToChooser(autoChooser);
    mechanismPoseLogger = new MechanismPoseLogger(groundIntake, indexer, shooter);
    manualButtonBindings();
  }

  public void manualButtonBindings() {
    // drivetrain controls
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () ->
                -driverController.getLeftY()
                    * Math.pow(Math.abs(driverController.getLeftY()), 1.2 - 1),
            () ->
                -driverController.getLeftX()
                    * Math.pow(Math.abs(driverController.getLeftX()), 1.2 - 1),
            () -> (0.5) * -driverController.getRightX()));
    driverController.y().onTrue(drive.resetGyro());
    // driverController
    //     .b()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               Time t =
    //                   ProjectileTrajectoryUtils.calcTargetTime(
    //                       MetersPerSecond.of(0),
    //                       MetersPerSecond.of(0),
    //                       new Translation3d(3, 4, 2.0),
    //                       Degrees.of(60));
    //               Logger.recordOutput("QuarticSolution/Time", t.in(Seconds));
    //             }));

    // driver indexer controls
    driverController
        .rightBumper()
        .whileTrue(
            indexer.setIndexerVelocity(
                () -> IndexerConstants.INDEXER_INTAKE_VELOCITY)) // TODO: update value later
        .onFalse(indexer.setIndexerVelocity(() -> IndexerConstants.INDEXER_NO_VELOCITY));

    // driver shooter controls
    driverController
        .rightTrigger()
        .whileTrue(
            shooter.setShooterVelocity(
                () -> ShooterConstants.SHOOTER_HIGH_GOAL_VELOCITY,
                () ->
                    ShooterConstants.SHOOTER_HIGH_GOAL_VELOCITY
                        .unaryMinus())) // TODO: update value later to shoot in
        // high goal
        .onFalse(shooter.setShooterVelocity(() -> ShooterConstants.SHOOTER_NO_VELOCITY));
    driverController
        .leftTrigger()
        .whileTrue(
            shooter.setShooterVelocity(
                () -> ShooterConstants.SHOOTER_LOW_GOAL_VELOCITY,
                () ->
                    ShooterConstants.SHOOTER_LOW_GOAL_VELOCITY
                        .unaryMinus())) // TODO: update value later to shoot in
        // low goal
        .onFalse(shooter.setShooterVelocity(() -> ShooterConstants.SHOOTER_NO_VELOCITY));

    // operator one button intake
    operatorController
        .x()
        .whileTrue(
            new ParallelCommandGroup(
                groundIntake
                    .setGroundIntakeRollerVelocity(
                        () ->
                            GroundIntakeConstants.GroundIntakeRollerConstants
                                .GROUNDINTAKE_ROLLER_VELOCITY
                                .times(-0.6))
                    .repeatedly(),
                indexer.setIndexerVelocity(() -> IndexerConstants.INDEXER_INTAKE_VELOCITY),
                shooter.setShooterVelocity(
                    () -> ShooterConstants.SHOOTER_REVERSE_VELOCITY,
                    () -> ShooterConstants.SHOOTER_REVERSE_VELOCITY.unaryMinus())))
        .onFalse(
            new ParallelCommandGroup(
                groundIntake.setGroundIntakeRollerVelocity(() -> RotationsPerSecond.of(0)),
                indexer.setIndexerVelocity(() -> IndexerConstants.INDEXER_NO_VELOCITY),
                shooter.setShooterVelocity(() -> ShooterConstants.SHOOTER_NO_VELOCITY)));

    // operator indexer controls
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
                () -> ShooterConstants.SHOOTER_REVERSE_VELOCITY,
                () ->
                    ShooterConstants.SHOOTER_REVERSE_VELOCITY
                        .unaryMinus())) // TODO: update value later
        .onFalse(shooter.setShooterVelocity(() -> ShooterConstants.SHOOTER_NO_VELOCITY));
    operatorController
        .rightTrigger()
        .whileTrue(
            shooter.setShooterVelocity(
                () -> ShooterConstants.SHOOTER_HIGH_GOAL_VELOCITY,
                () ->
                    ShooterConstants.SHOOTER_HIGH_GOAL_VELOCITY
                        .unaryMinus())) // TODO: update value later to shoot in
        // high goal
        .onFalse(shooter.setShooterVelocity(() -> ShooterConstants.SHOOTER_NO_VELOCITY));
    operatorController
        .b()
        .whileTrue(
            shooter.setShooterVelocity(
                () -> ShooterConstants.SHOOTER_LOW_GOAL_VELOCITY,
                () ->
                    ShooterConstants.SHOOTER_LOW_GOAL_VELOCITY
                        .unaryMinus())) // TODO: update value later to shoot in
        // low goal
        .onFalse(shooter.setShooterVelocity(() -> ShooterConstants.SHOOTER_NO_VELOCITY));

    // ground intake controls
    operatorController
        .y()
        .whileTrue(
            groundIntake.setGroundIntakePivotVoltage(
                () -> GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_VOLTAGE_UP))
        .onFalse(groundIntake.setGroundIntakePivotVoltage(() -> Volts.of(0)));
    operatorController
        .a()
        .whileTrue(
            groundIntake.setGroundIntakePivotVoltage(
                () -> GroundIntakePivotConstants.GROUND_INTAKE_PIVOT_VOLTAGE_DOWN))
        .onFalse(groundIntake.setGroundIntakePivotVoltage(() -> Volts.of(0)));
    groundIntake.setDefaultCommand(
        groundIntake.setGroundIntakeRollerVelocity(
            () ->
                GroundIntakeRollerConstants.GROUNDINTAKE_ROLLER_VELOCITY.times(
                    operatorController.getLeftY()))); // TODO: update value later
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
    // Logger.recordOutput(
    // "FieldSimulation/Lunites",
    // SimulatedArena.getInstance().getGamePiecesArrayByType("Lunite"));
  }
}
