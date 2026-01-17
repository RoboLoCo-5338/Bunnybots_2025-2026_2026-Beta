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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.util.ProjectileTrajectoryUtils;
import frc.robot.util.ProjectileTrajectoryUtils.FixedTrajectorySolution;
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
            shooter.setShooterVelocityCommand(
                () -> ShooterConstants.SHOOTER_HIGH_GOAL_VELOCITY,
                () -> ShooterConstants.SHOOTER_HIGH_GOAL_VELOCITY.unaryMinus()),
            new WaitCommand(1.0),
            indexer.setIndexerVelocity(() -> RotationsPerSecond.of(3)),
            groundIntake.setGroundIntakeRollerVelocity(() -> RotationsPerSecond.of(3))));
    NamedCommands.registerCommand(
        "Shoot Low",
        new SequentialCommandGroup(
            shooter.setShooterVelocityCommand(
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
            shooter.setShooterVelocityCommand(() -> RotationsPerSecond.of(0)),
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
            shooter.setShooterVelocityCommand(
                () -> ShooterConstants.SHOOTER_HIGH_GOAL_VELOCITY,
                () -> ShooterConstants.SHOOTER_HIGH_GOAL_VELOCITY.unaryMinus()),
            new WaitCommand(1.0),
            indexer.setIndexerVelocity(() -> RotationsPerSecond.of(3)),
            groundIntake.setGroundIntakeRollerVelocity(() -> RotationsPerSecond.of(3))));
    autoChooser.addOption(
        "No Leave Low Outer 3",
        new SequentialCommandGroup(
            shooter.setShooterVelocityCommand(
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
    initializeTunables();
  }

  private ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");
  private GenericEntry kPEntry;
  private GenericEntry kIEntry;
  private GenericEntry kShooterEntry;
  private GenericEntry kDisplacementXEntry;

  public void initializeTunables() {
    // Create entries for Kp and Ki, with default values
    kPEntry =
        tuningTab
            .add("Kp", 0.01) // Key "Kp", default 0.01
            .withWidget("NumberSlider") // Use a slider widget
            .getEntry();
    kIEntry =
        tuningTab
            .add("Ki", 0.001) // Key "Ki", default 0.001
            .withWidget("NumberSlider")
            .getEntry();
    kShooterEntry =
        tuningTab
            .add("Kshooter", 5.13) // Key "Ki", default 0.001
            .withWidget("NumberSlider")
            .getEntry();
    kDisplacementXEntry =
        tuningTab
            .add("displacementX", 2.0) // Key "Ki", default 0.001
            .withWidget("NumberSlider")
            .getEntry();
  }

  // In your periodic methods or wherever you use them:
  public double getKp() {
    return kPEntry.getDouble(0.01); // Get value, use default if missing
  }

  public double getKi() {
    return kIEntry.getDouble(0.001);
  }

  public double getKshooter() {
    return kShooterEntry.getDouble(5.13);
  }

  public double getDisplacementX() {
    Logger.recordOutput("displacementX", kDisplacementXEntry.getDouble(67));
    return kDisplacementXEntry.getDouble(2.0);
  }

  private static final Translation3d hubLocation = new Translation3d(0, 0, 2);
  private static final Translation3d shooterOffset =
      new Translation3d(-Inches.of(0.1956095).in(Meters), 0, Inches.of(16.081505).in(Meters));
  private static final Angle shooterAltitude = Degrees.of(50);

  private static AngularVelocity lastOmega = RadiansPerSecond.of(0);
  private static int numLagFrames = 3;

  private static double deadzone(double input, double zone) {
    if (Math.abs(input) < zone) {
      return 0.0;
    } else {
      return input;
    }
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

    driverController
        .a()
        .whileTrue(
            Commands.run(
                () -> {
                  double start = HALUtil.getFPGATime();

                  // TODO: essentialy euler's method, tracking position, velocity, angle, angular
                  // velocity,
                  // capping actual acceleration & angular acceleration by computed max values
                  // Translation2d fieldPos = drive.getPose().getTranslation();
                  Translation3d // targetDisplacement =
                      // shooterOffset.plus(new Translation3d(fieldPos.getX(), fieldPos.getY(), 0));
                      // lunar converter 152cm bottom - 203cm top
                      targetDisplacement =
                          new Translation3d(-getDisplacementX(), 0, (2.03 + 1.52) / 2)
                              .minus(shooterOffset);

                  FixedTrajectorySolution solution =
                      ProjectileTrajectoryUtils.calcFiringSolution(
                          MetersPerSecond.of(0),
                          MetersPerSecond.of(0),
                          targetDisplacement,
                          shooterAltitude);

                  // double shooterSpeedTransfer =
                  //     ProjectileSpeedUtils.calcSpeedTransferPercentage(
                  //         ShooterConstants.ShooterSimConstants.SHOOTER_MOI,
                  //         Pounds.of(0.2),
                  //         Inches.of(3.0 / 2));
                  //   shooterSpeedTransfer *= getKshooter();
                  AngularVelocity shooterAngularVelocity =
                      RadiansPerSecond.of(
                          solution.shooterVelocity.in(MetersPerSecond) * getKshooter());

                  Logger.recordOutput(
                      "DriveTest/ShooterRotationsPerSecond",
                      shooterAngularVelocity.in(RotationsPerSecond));
                  shooter.setShooterVelocity(shooterAngularVelocity);

                  Angle azimuthDiff =
                      Radians.of(
                          deadzone(
                              (solution.azimuth.in(Radians)
                                  - drive.getPose().getRotation().getRadians()),
                              0.05));

                  AngularVelocity omega = azimuthDiff.div(Seconds.of(1.0));
                  if (Math.abs(omega.in(RadiansPerSecond)) > drive.getMaxAngularSpeedRadPerSec()) {
                    omega =
                        RadiansPerSecond.of(
                            Math.copySign(
                                drive.getMaxAngularSpeedRadPerSec(), omega.in(RadiansPerSecond)));
                  }
                  drive.runVelocity(
                      new ChassisSpeeds(MetersPerSecond.of(0), MetersPerSecond.of(0), omega));
                  Logger.recordOutput("DriveTest/AngularSpeed", omega.in(RadiansPerSecond));
                  lastOmega = omega;

                  Logger.recordOutput("DriveTest/azimuthDiff", azimuthDiff.in(Degrees));
                },
                drive,
                shooter));

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
            shooter.setShooterVelocityCommand(
                () -> ShooterConstants.SHOOTER_HIGH_GOAL_VELOCITY,
                () ->
                    ShooterConstants.SHOOTER_HIGH_GOAL_VELOCITY
                        .unaryMinus())) // TODO: update value later to shoot in
        // high goal
        .onFalse(shooter.setShooterVelocityCommand(() -> ShooterConstants.SHOOTER_NO_VELOCITY));
    driverController
        .leftTrigger()
        .whileTrue(
            shooter.setShooterVelocityCommand(
                () -> ShooterConstants.SHOOTER_LOW_GOAL_VELOCITY,
                () ->
                    ShooterConstants.SHOOTER_LOW_GOAL_VELOCITY
                        .unaryMinus())) // TODO: update value later to shoot in
        // low goal
        .onFalse(shooter.setShooterVelocityCommand(() -> ShooterConstants.SHOOTER_NO_VELOCITY));

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
                shooter.setShooterVelocityCommand(
                    () -> ShooterConstants.SHOOTER_REVERSE_VELOCITY,
                    () -> ShooterConstants.SHOOTER_REVERSE_VELOCITY.unaryMinus())))
        .onFalse(
            new ParallelCommandGroup(
                groundIntake.setGroundIntakeRollerVelocity(() -> RotationsPerSecond.of(0)),
                indexer.setIndexerVelocity(() -> IndexerConstants.INDEXER_NO_VELOCITY),
                shooter.setShooterVelocityCommand(() -> ShooterConstants.SHOOTER_NO_VELOCITY)));

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
            shooter.setShooterVelocityCommand(
                () -> ShooterConstants.SHOOTER_REVERSE_VELOCITY,
                () ->
                    ShooterConstants.SHOOTER_REVERSE_VELOCITY
                        .unaryMinus())) // TODO: update value later
        .onFalse(shooter.setShooterVelocityCommand(() -> ShooterConstants.SHOOTER_NO_VELOCITY));
    operatorController
        .rightTrigger()
        .whileTrue(
            shooter.setShooterVelocityCommand(
                () -> ShooterConstants.SHOOTER_HIGH_GOAL_VELOCITY,
                () ->
                    ShooterConstants.SHOOTER_HIGH_GOAL_VELOCITY
                        .unaryMinus())) // TODO: update value later to shoot in
        // high goal
        .onFalse(shooter.setShooterVelocityCommand(() -> ShooterConstants.SHOOTER_NO_VELOCITY));
    operatorController
        .b()
        .whileTrue(
            shooter.setShooterVelocityCommand(
                () -> ShooterConstants.SHOOTER_HIGH_GOAL_VELOCITY,
                () ->
                    ShooterConstants.SHOOTER_HIGH_GOAL_VELOCITY
                        .unaryMinus())) // TODO: update value later to shoot in
        // low goal
        .onFalse(shooter.setShooterVelocityCommand(() -> ShooterConstants.SHOOTER_NO_VELOCITY));

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
