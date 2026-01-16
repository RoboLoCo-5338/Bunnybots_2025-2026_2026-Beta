package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
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
import frc.robot.util.ProjectileTrajectoryUtils.FixedTrajectorySolution;
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
    kAutoShootAccelTest,
    kAutoDriveAccelTest,

    kManualScore,

    kAutoDefault,
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
    periodicHash.put(
        RobotAction.kTeleopDefault,
        () -> {
          // ProjectileTrajectoryUtils.testDerivative(
          //     MetersPerSecond.of(1),
          //     MetersPerSecond.of(0),
          //     new Translation3d(3, 4, 2.0),
          //     Degrees.of(60));
        });
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
                  Pounds.of(0.2),
                  Inches.of(3.0 / 2));
          Logger.recordOutput(
              "Trajectory/ShooterRotationsPerSecond",
              shooterAngularVelocity.in(RotationsPerSecond));
          m_shooter.setShooterVelocity(shooterAngularVelocity);
        });
    periodicHash.put(RobotAction.kAutoDriveTest, this::autoDriveTestPeriodic);
    periodicHash.put(RobotAction.kAutoShootAccelTest, this::autoShooterAccelTest);
    periodicHash.put(RobotAction.kAutoDriveAccelTest, this::autoDriveAccelTest);
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

  Time actionStart;

  public void updateRobotAction(RobotAction action) {
    /*
     *  FRC team 422 MechTech Dragons - may end up using this code
     */
    if (action != m_profiles.getCurrentProfile()) {
      m_profiles.setCurrentProfile(action);
      actionStart = Microseconds.of(HALUtil.getFPGATime());
      lastPeriodic = Microseconds.of(0);
    }
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

  private AngularVelocity lastOmega = RadiansPerSecond.of(0);
  private int numLagFrames = 3;

  private double deadzone(double input, double zone) {
    if (Math.abs(input) < zone) {
      return 0.0;
    } else {
      return input;
    }
  }

  public void autoDriveTestPeriodic() {
    double start = HALUtil.getFPGATime();

    // TODO: essentialy euler's method, tracking position, velocity, angle, angular velocity,
    // capping actual acceleration & angular acceleration by computed max values
    Translation2d fieldPos = m_drive.getPose().getTranslation();
    Translation3d targetDisplacement =
        shooterOffset.plus(new Translation3d(fieldPos.getX(), fieldPos.getY(), 0));
    targetDisplacement = new Translation3d(3, 4, 2);

    FixedTrajectorySolution solution =
        ProjectileTrajectoryUtils.calcFiringSolution(
            MetersPerSecond.of(0), MetersPerSecond.of(0), targetDisplacement, shooterAltitude);

    AngularVelocity shooterAngularVelocity =
        ProjectileSpeedUtils.calcNecessaryWheelSpeed(
            solution.shooterVelocity,
            ShooterConstants.ShooterSimConstants.SHOOTER_MOI,
            Pounds.of(0.2),
            Inches.of(3.0 / 2));
    Logger.recordOutput(
        "DriveTest/ShooterRotationsPerSecond", shooterAngularVelocity.in(RotationsPerSecond));
    m_shooter.setShooterVelocityCommand(() -> shooterAngularVelocity, () -> shooterAngularVelocity);

    Angle azimuthDiff =
        Radians.of((solution.azimuth.in(Radians) - m_drive.getPose().getRotation().getRadians()));

    AngularVelocity currentOmega =
        RadiansPerSecond.of(m_drive.getChassisSpeeds().omegaRadiansPerSecond);
    Angle azimuthDiffFuture =
        Radians.of(
            deadzone(
                solution.azimuth.in(Radians)
                    - (m_drive.getPose().getRotation().getRadians()
                        + currentOmega.times(Milliseconds.of(20).times(numLagFrames)).in(Radians)),
                0.05));

    AngularVelocity omega = azimuthDiffFuture.div(Milliseconds.of(20).times(numLagFrames));
    if (Math.abs(omega.in(RadiansPerSecond)) > m_drive.getMaxAngularSpeedRadPerSec()) {
      omega =
          RadiansPerSecond.of(
              Math.copySign(m_drive.getMaxAngularSpeedRadPerSec(), omega.in(RadiansPerSecond)));
    }
    m_drive.runVelocity(new ChassisSpeeds(MetersPerSecond.of(0), MetersPerSecond.of(0), omega));
    Logger.recordOutput("DriveTest/AngularSpeed", omega.in(RadiansPerSecond));
    lastOmega = omega;

    Logger.recordOutput("DriveTest/azimuthDiff", azimuthDiff.in(Degrees));
  }

  Time lastPeriodic;

  private double magicFunctionAccelTest(double t) {
    /* shooterAccelTest will attempt to match x(t)=e^t */
    return 10
        + (t) * (t - 1.5) * (t - 4) * (t - 9) * (t - 11) * (t - 13) * (t - 14) * (t - 17) * (t - 19)
            / 1000000.0;
    // return Math.pow(Math.E, t - 5);
  }

  public void autoShooterAccelTest() {
    Time now = Microseconds.of(HALUtil.getFPGATime());
    Time dTime = now.minus(lastPeriodic);
    Time t = now.minus(actionStart);
    Logger.recordOutput("ShooterTest/t", t.in(Seconds));
    AngularVelocity x = RadiansPerSecond.of(magicFunctionAccelTest(t.in(Seconds)));

    Logger.recordOutput("ShooterTest/x", x.in(RadiansPerSecond));
    AngularVelocity xIn = RadiansPerSecond.of(magicFunctionAccelTest(t.in(Seconds) + 0.1));
    Logger.recordOutput("ShooterTest/xIn", xIn.in(RadiansPerSecond));
    xIn = xIn.times(1.6454648659);
    m_shooter.setShooterVelocity(xIn);
    // x = x.times(1.5027290797);
    // x = x.times(1.6201884787);
    // x = x.times(1.6885619176);
    // x = x.times(1.6338803120);

    Logger.recordOutput(
        "ShooterTest/dx",
        magicFunctionAccelTest(t.in(Seconds))
            - magicFunctionAccelTest((lastPeriodic.minus(actionStart)).in(Seconds)));

    Logger.recordOutput("ShooterTest/dt", dTime.in(Seconds));
    Logger.recordOutput(
        "ShooterTest/dxdt",
        (magicFunctionAccelTest(t.in(Seconds))
                - magicFunctionAccelTest((lastPeriodic.minus(actionStart)).in(Seconds)))
            / dTime.in(Seconds));

    lastPeriodic = now;
  }

  private double magicFunctionDriveAccelTest(double t) {
    /* shooterAccelTest will attempt to match x(t)=e^t */
    t /= 5.0;
    return ((t) * (t - 1.5) * (t - 4) * (t - 9) * (t - 11) * (t - 13) * (t - 14) * (t - 17)
            * (t - 19) / 1000000.0)
        / 5.0;
    // return Math.pow(Math.E, t - 5);
  }

  private double magicFunctionDriveAccelTestIntegral(double t1, double t2) {
    int n = 100; // Increase number of intervals for better accuracy
    double h = (t2 - t1) / n;
    double sum = 0.5 * (magicFunctionDriveAccelTest(t1) + magicFunctionDriveAccelTest(t2));
    for (int i = 1; i < n; i++) {
      double ti = t1 + i * h;
      sum += magicFunctionDriveAccelTest(ti);
    }
    return sum * h;
  }

  private LinearVelocity maxLinearVelocityPerAngularVelocity(AngularVelocity omega) {
    // Simple model: assume a max linear velocity of 3 m/s at 0 rad/s angular velocity,
    // linearly decreasing to 0 m/s at max angular velocity of the robot.
    AngularVelocity maxAngularVelocityRadPerSec =
        RadiansPerSecond.of(m_drive.getMaxAngularSpeedRadPerSec());
    double ratio =
        1 - Math.abs(omega.in(RadiansPerSecond)) / maxAngularVelocityRadPerSec.in(RadiansPerSecond);
    ratio = Math.max(0, ratio); // Ensure non-negative
    return MetersPerSecond.of(3 * ratio);
  }

  private Distance wheelBaseRadius = Meters.of(m_drive.DRIVE_BASE_RADIUS);

  /**
   * Given an angular velocity omega and an offsetY (lateral offset from the robot's center),
   * calculates the tangential linear velocity at that offset. Useful for when the shooter is at one
   * end of the robot, and we want to rotate around the center of the shooter.
   *
   * @param omega The angular velocity of the robot.
   * @param offsetY The lateral offset from the robot's center to the point of interest.
   * @return The tangential linear velocity at the given offset.
   */
  private LinearVelocity tangentialVelocity(AngularVelocity omega, Distance offsetY) {
    return MetersPerSecond.of(offsetY.in(Meters) * omega.in(RadiansPerSecond));
  }

  double startY = 0.442189;

  public void autoDriveAccelTest() {
    Time now = Microseconds.of(HALUtil.getFPGATime());
    Time dTime = now.minus(lastPeriodic);
    Time t = now.minus(actionStart);
    Logger.recordOutput("DriveTest/t", t.in(Seconds));
    AngularVelocity x = RadiansPerSecond.of(magicFunctionDriveAccelTest(t.in(Seconds)));

    Logger.recordOutput("DriveTest/x", x.in(RadiansPerSecond));
    AngularVelocity xIn = RadiansPerSecond.of(magicFunctionDriveAccelTest(t.in(Seconds) + 0.1));
    // xIn = RadiansPerSecond.of();
    Logger.recordOutput("DriveTest/xIn", xIn.in(RadiansPerSecond));
    xIn = xIn.times(1 / 3.5857681131);
    // x = x.times(1.5027290797);
    // x = x.times(1.6201884787);
    // x = x.times(1.6885619176);
    // x = x.times(1.6338803120);

    double diff = startY - m_drive.getPose().getTranslation().getY();
    m_drive.runVelocityDangerous(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(MetersPerSecond.of(2), MetersPerSecond.of(diff / 2), xIn),
            m_drive
                .getRotation()
                .plus(
                    new Rotation2d(
                        Radians.of(
                            1.0
                                * magicFunctionDriveAccelTestIntegral(
                                    t.in(Seconds) + 0.0, t.in(Seconds) + 0.02)
                                / 2)))));

    Logger.recordOutput(
        "DriveTest/dx",
        magicFunctionDriveAccelTest(t.in(Seconds))
            - magicFunctionDriveAccelTest((lastPeriodic.minus(actionStart)).in(Seconds)));

    Logger.recordOutput("DriveTest/dt", dTime.in(Seconds));
    Logger.recordOutput(
        "DriveTest/dxdt",
        (magicFunctionDriveAccelTest(t.in(Seconds))
                - magicFunctionDriveAccelTest((lastPeriodic.minus(actionStart)).in(Seconds)))
            / dTime.in(Seconds));

    lastPeriodic = now;
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
