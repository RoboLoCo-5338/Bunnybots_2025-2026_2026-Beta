package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.ProjectileSpeedUtils;
import frc.robot.util.ProjectileTrajectoryUtils;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class RobotState {

  private Drive m_drive;
  private GroundIntake m_intake;
  private Shooter m_shooter;
  private Indexer m_indexer;

  private Vision m_vision;

  public enum RobotAction {
    kTeleopDefault,
    kLuniteIntaking,
    kLuniteLowGoal,
    kLuniteHighGoal,
    kDriveIntakeUp,

    kAutoScore,
    kAutoDriveTest,

    kManualScore,

    kAutoDefault,
    kAutoAutoScore,
    kAutoLuniteIntaking,
    kAutoLuniteLowGoal,
    kAutoLuniteHighGoal,
    kAutoDriveIntakeUp,
  }

  private SubsystemProfiles<RobotAction> m_profiles;

  private Timer m_timer = new Timer();
  private Timer m_secondaryTimer = new Timer();
  private Timer m_tertiaryTimer = new Timer();
  private double m_odometryTrustFactor = 0.0;
  private boolean m_autoTestingMode = false;

  private Timer m_threadPriorityTimer = new Timer();

  //   private final List<RobotAction> kAlgaeDescoreSequence =
  //     List.of(
  //       RobotAction.blah
  //       RobotAction.blahblah;

  //   private final List<RobotAction> kAlgaeDescoreSequenceAuto =
  //     List.of(
  //       RobotAction.kAlgaeDescoringInitial,
  //       RobotAction.kAlgaeDescoringDeployManipulator,
  //       RobotAction.kAlgaeDescoringMoveUp,
  //       RobotAction.kAlgaeDescoringDriveAway);

  // Singleton logic
  private static RobotState m_instance;

  private RobotState(
      Drive drive, GroundIntake intake, Shooter shooter, Indexer indexer, Vision vision) {
    m_drive = drive;
    m_intake = intake;
    m_shooter = shooter;
    m_indexer = indexer;
    m_vision = vision;

    Map<RobotAction, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(RobotAction.kTeleopDefault, () -> {});
    periodicHash.put(
        RobotAction.kAutoScore,
        () -> {
          Time timeOfFlight =
              ProjectileTrajectoryUtils.calcTargetTime(
                  MetersPerSecond.of(0),
                  MetersPerSecond.of(0),
                  new Translation3d(3, 4, 2.0),
                  Degrees.of(60));
          Logger.recordOutput("Trajectory/Time", timeOfFlight.in(Seconds));
          LinearVelocity shooterVelocity =
              ProjectileTrajectoryUtils.calcShooterVelocity(
                  timeOfFlight, new Translation3d(3, 4, 2.0), Degrees.of(60));
          Logger.recordOutput("Trajectory/Velocity", shooterVelocity.in(MetersPerSecond));
          Angle robotHeadingAzimuth =
              ProjectileTrajectoryUtils.calcRobotHeadingAzimuth(
                  MetersPerSecond.of(0),
                  MetersPerSecond.of(0),
                  timeOfFlight,
                  shooterVelocity,
                  new Translation3d(3, 4, 2.0),
                  Degrees.of(60));
          Logger.recordOutput("Trajectory/Azimuth", robotHeadingAzimuth.in(Degrees));

          AngularVelocity shooterAngularVelocity =
              ProjectileSpeedUtils.calcNecessaryWheelSpeed(
                  shooterVelocity,
                  ShooterConstants.ShooterSimConstants.SHOOTER_MOI,
            Pounds.of(0.1),
                  Inches.of(3.0 / 2));
          Logger.recordOutput(
              "Trajectory/ShooterRotationsPerSecond",
              shooterAngularVelocity.in(RotationsPerSecond));
          m_shooter.setShooterVelocity(() -> shooterAngularVelocity);
        });
    periodicHash.put(RobotAction.kAutoDriveTest, this::autoDriveTestPeriodic);
    periodicHash.put(RobotAction.kAutoDefault, () -> {});
    // periodicHash.put(RobotAction.kLuniteLowGoal, this::luniteLowGoalPeriodic);
    // periodicHash.put(RobotAction.kLuniteHighGoal, this::luniteLowGoalPeriodic);
    // periodicHash.put(RobotAction.kLuniteIntaking, this::coralIntakingPeriodic);
    // periodicHash.put(RobotAction.kAutoScore, this::autoScorePeriodic);
    // periodicHash.put(RobotAction.kManualScore, this::manualScorePeriodic);
    // periodicHash.put(RobotAction.kDriveToProcessor, this::driveToProcessorPeriodic);
    // periodicHash.put(RobotAction.kCoralOuttaking, this::coralOuttakingPeriodic);
    // periodicHash.put(RobotAction.kProcessorOuttake, () -> {});
    // periodicHash.put(RobotAction.kBargeScore, this::bargeScorePeriodic);
    // periodicHash.put(RobotAction.kBargeAutoScore, this::bargeAutoScorePeriodic);
    // periodicHash.put(RobotAction.kCoralEject, () -> {});
    // periodicHash.put(RobotAction.kLollipopIntake, this::lollipopIntakePeriodic);
    // periodicHash.put(RobotAction.kCoralOTB, this::coralIntakingPeriodic);
    // periodicHash.put(RobotAction.kCoralOTBL1, this::coralOTBL1Periodic);
    // periodicHash.put(RobotAction.kAlgaeDescoringInitial, this::algaeDescoringPeriodic);
    // periodicHash.put(RobotAction.kAlgaeDescoringDeployManipulator, this::algaeDescoringPeriodic);
    // periodicHash.put(RobotAction.kAlgaeDescoringMoveUp, this::algaeDescoringPeriodic);
    // periodicHash.put(RobotAction.kAlgaeDescoringDriveAway, this::algaeDescoringPeriodic);
    // periodicHash.put(RobotAction.kAlgaeDescoringFinal, this::algaeDescoringPeriodic);
    // periodicHash.put(RobotAction.kAutoAutoScore, this::autoAutoScorePeriodic);
    // periodicHash.put(RobotAction.kAutoCoralIntaking, this::autoCoralIntakingPeriodic);
    // periodicHash.put(RobotAction.kAutoCoralOuttaking, this::coralOuttakingPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, RobotAction.kTeleopDefault);

    m_threadPriorityTimer.start();
  }

  public void updateRobotAction(RobotAction action) {
    /*
     *  FRC team 422 MechTech Dragons - may end up using this code
     */
    // DriveProfiles newDriveState = DriveProfiles.kDefault;

    // switch (action) {
    //   case kAutoDefault:
    //     newDriveState = DriveProfiles.kAutoAlign;
    //     break;
    //   case kTeleopDefault:
    //     newDriveState = DriveProfiles.kDefault;
    //     break;
    //   default:
    //     break;
    // }

    // if (newDriveState != m_drive.getCurrentProfile()) {
    //   m_drive.updateProfile(newDriveState);
    // }

    m_profiles.setCurrentProfile(action);
  }

  public static RobotState startInstance(
      Drive drive, GroundIntake intake, Shooter shooter, Indexer indexer, Vision vision) {
    if (m_instance == null) {
      m_instance = new RobotState(drive, intake, shooter, indexer, vision);
    }
    return m_instance;
  }

  public static RobotState getInstance() {
    return m_instance;
  }

  private final CommandXboxController driverController = new CommandXboxController(0);

  private final Translation3d hubLocation = new Translation3d(0, 0, 2);
  private final Translation3d shooterOffset =
      new Translation3d(0, -Inches.of(0.1956095).in(Meters), Inches.of(16.081505).in(Meters));
  private final Angle shooterAltitude = Degrees.of(60);

  public void autoDriveTestPeriodic() {
    double start = HALUtil.getFPGATime();

    // TODO: essentialy euler's method, tracking position, velocity, angle, angular velocity,
    // capping actual acceleration & angular acceleration by computed max values
    Translation2d fieldPos = m_drive.getPose().getTranslation();
    Translation3d targetDisplacement =
        shooterOffset.plus(new Translation3d(fieldPos.getX(), fieldPos.getY(), 0));
    targetDisplacement = new Translation3d(3, 4, 2);
    Time timeOfFlight =
        ProjectileTrajectoryUtils.calcTargetTime(
            MetersPerSecond.of(0), MetersPerSecond.of(0), targetDisplacement, shooterAltitude);
    Logger.recordOutput("DriveTest/Trajectory/Time", timeOfFlight.in(Seconds));
    LinearVelocity shooterVelocity =
        ProjectileTrajectoryUtils.calcShooterVelocity(
            timeOfFlight, new Translation3d(3, 4, 2.0), Degrees.of(60));
    Logger.recordOutput("DriveTest/Trajectory/Velocity", shooterVelocity.in(MetersPerSecond));
    Angle robotHeadingAzimuth =
        ProjectileTrajectoryUtils.calcRobotHeadingAzimuth(
            MetersPerSecond.of(0),
            MetersPerSecond.of(0),
            timeOfFlight,
            shooterVelocity,
            targetDisplacement,
            shooterAltitude);
    Logger.recordOutput("DriveTest/Trajectory/Azimuth", robotHeadingAzimuth.in(Degrees));

    AngularVelocity shooterAngularVelocity =
        ProjectileSpeedUtils.calcNecessaryWheelSpeed(
            shooterVelocity,
            ShooterConstants.ShooterSimConstants.SHOOTER_MOI,
            Pounds.of(0.1),
            Inches.of(3.0 / 2));
    Logger.recordOutput(
        "DriveTest/ShooterRotationsPerSecond", shooterAngularVelocity.in(RotationsPerSecond));
    m_shooter.setShooterVelocity(() -> {return shooterAngularVelocity;});
    AngularVelocity azimuthSpeed =
        RadiansPerSecond.of(
            (robotHeadingAzimuth.in(Radians) - m_drive.getPose().getRotation().getRadians()) * 1);

    Logger.recordOutput("DriveTest/azimuthSpeed", azimuthSpeed.in(DegreesPerSecond));
    if (azimuthSpeed.in(RadiansPerSecond) > m_drive.getMaxAngularSpeedRadPerSec()) {
      m_drive.runVelocity(
          new ChassisSpeeds(
              MetersPerSecond.of(0),
              MetersPerSecond.of(0),
              RadiansPerSecond.of(m_drive.getMaxAngularSpeedRadPerSec())));
    } else {
      m_drive.runVelocity(
          new ChassisSpeeds(MetersPerSecond.of(0), MetersPerSecond.of(0), azimuthSpeed));
    }
  }

  public void onEnable() {
    m_drive.stop();

    // setDefaultAction();

    // m_numLowGoalScoredAuto = 0;
    // m_numHighGoalScoredAuto = 0;
  }

  public void onDisable() {
    m_drive.stop();
  }

  public void updateRobotState() {
    double start = HALUtil.getFPGATime();

    // Pose2d[] hardcoded =
    //   new Pose2d[] {
    //     new Pose2d(12.92, 1.96, Rotation2d.fromDegrees(90)),
    //     new Pose2d(12.92, 5.99, Rotation2d.fromDegrees(270)),
    //     new Pose2d(4.3519, 5.99, Rotation2d.fromDegrees(270)),
    //     new Pose2d(4.3519, 1.96, Rotation2d.fromDegrees(90)),
    //     new Pose2d(15.92385, 0.9075, Rotation2d.fromDegrees(126)),
    //     new Pose2d(
    //       15.92385, 7.1825, Rotation2d.fromDegrees(54).plus(Rotation2d.fromDegrees(180))),
    //     new Pose2d(1.6244, 7.1825, Rotation2d.fromDegrees(-54)),
    //     new Pose2d(
    //       1.6244, 0.9075, Rotation2d.fromDegrees(-126).plus(Rotation2d.fromDegrees(180))),
    //   };

    // Logger.recordOutput("Hardcoded", hardcoded);

    if (m_threadPriorityTimer != null && m_threadPriorityTimer.hasElapsed(20)) {
      Threads.setCurrentThreadPriority(true, 10);
      m_threadPriorityTimer = null;
    }
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    m_profiles.getPeriodicFunctionTimed().run();

    // if (Constants.kUseComponents) {
    //   updateComponent();
    // }

    // updateLED();

    // SetpointGenerator.logAutoIntakeLines();

    // Logger.recordOutput("RobotState/CurrentAction", m_profiles.getCurrentProfile());
    // Logger.recordOutput("RobotState/TimerValue", m_timer.get());
    // Logger.recordOutput("RobotState/SecondaryTimerValue", m_secondaryTimer.get());
    // Logger.recordOutput("RobotState/TertiaryTimerValue", m_tertiaryTimer.get());

    // Logger.recordOutput("RobotState/UsingVision", getUsingVision());

    // Logger.recordOutput("OdometryTrustFactor", m_odometryTrustFactor);

    // Logger.recordOutput("RobotState/NumCoralScoredAuto", m_numCoralScoredAuto);

    // Logger.recordOutput("RobotState/HasRunASingleCycle", m_hasRunASingleCycle);
    // Logger.recordOutput("RobotState/HasRunTwoCycle", m_hasRunTwoCycles);

    // Logger.recordOutput("PeriodicTime/RobotState", (HALUtil.getFPGATime() - start) / 1000.0);
  }
}
