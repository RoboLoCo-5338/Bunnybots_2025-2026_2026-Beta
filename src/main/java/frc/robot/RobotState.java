package frc.robot;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;

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

  public void coralIntakingPeriodic() {
    // when we have a game piece don't runm
    // if (m_manipulator.gamePieceInFunnel() || m_manipulator.fullyIndexed()) {
    //   if (!(DriverStation.isAutonomous() || m_autoTestingMode)) {
    //     m_drive.updateProfile(DriveProfiles.kDefault);
    //   }
    // }
    // if (m_manipulator.hasGamePiece()) {
    //   if (!m_gamepieceUpdate) {
    //     m_gamepieceUpdate = true;
    //     m_led.gamepiece();
    //   }
    //   m_indexer.updateState(IndexerState.kIdle);
    // }
    // // wait until the manipulator is in position before we intake
    // else if (m_manipulator.atSetpoint() && m_elevator.atSetpoint()) {
    //   m_indexer.updateState(IndexerState.kIndexing);
    // } else {
    //   m_indexer.updateState(IndexerState.kIdle);
    // }
  }

  public void onEnable() {
    // m_drive.setBrake();

    // setDefaultAction();

    // m_numLowGoalScoredAuto = 0;
    // m_numHighGoalScoredAuto = 0;
  }

  public void onDisable() {}

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
