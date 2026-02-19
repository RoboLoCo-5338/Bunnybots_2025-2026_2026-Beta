package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.nodes.jni.nodesJNI;
import com.nodes.jni.nodesJNI.TrajectorySolution;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import java.util.LinkedList;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AlignCommands {
  private static Time deltaT = Milliseconds.of(20);

  public static void initializeTunables() {
    initializeTurnPidEntries();
    initializeKShooter();
    initializeDisplacementEntries();
    initializeTemporalDisplacements();
    initializeLimits();
  }

  private static GenericEntry kPEntry;
  private static GenericEntry kIEntry;
  private static GenericEntry kDEntry;

  public static void initializeTurnPidEntries() {
    ShuffleboardTab tuningTab = Shuffleboard.getTab("AlignTurnPID");
    kPEntry =
        tuningTab
            .add("Kp", 2.7) // Key "Kp", default 0.01
            .withWidget("NumberSlider") // Use a slider widget
            .getEntry();
    kIEntry =
        tuningTab
            .add("Ki", 0.01) // Key "Ki", default 0.001
            .withWidget("NumberSlider")
            .getEntry();
    kDEntry =
        tuningTab
            .add("Kd", 0.4) // Key "Kd", default 0.001
            .withWidget("NumberSlider")
            .getEntry();
  }

  public static double getKp() {
    return kPEntry.getDouble(0.01);
  }

  public static double getKi() {
    return kIEntry.getDouble(0.001);
  }

  public static double getKd() {
    return kDEntry.getDouble(0.001);
  }

  private static GenericEntry kDisplacementXEntry;
  private static GenericEntry kDisplacementYEntry;
  private static GenericEntry kDisplacementZEntry;

  public static void initializeDisplacementEntries() {
    ShuffleboardTab tuningTab = Shuffleboard.getTab("HubDisplacement");
    kDisplacementXEntry =
        tuningTab
            .add(
                "displacementX",
                Inches.of((16 + 22.2)).in(Meters)) // bumper is 16, width of hub is 44.4
            .withWidget("NumberSlider")
            .getEntry();
    kDisplacementYEntry =
        tuningTab
            .add("displacementY", Inches.of(0).in(Meters))
            .withWidget("NumberSlider")
            .getEntry();
    kDisplacementZEntry =
        tuningTab
            .add("displacementZ", Inches.of(72).in(Meters)) // hub is 72 inches tall
            .withWidget("NumberSlider")
            .getEntry();
  }

  public static double getDisplacementX() {
    return kDisplacementXEntry.getDouble(Inches.of((16 + 22.2)).in(Meters));
  }

  public static double getDisplacementY() {
    return kDisplacementYEntry.getDouble(Inches.of(0).in(Meters));
  }

  public static double getDisplacementZ() {
    return kDisplacementZEntry.getDouble(Inches.of(72).in(Meters));
  }

  private static GenericEntry kShooterEntry;

  public static void initializeKShooter() {
    ShuffleboardTab tuningTab = Shuffleboard.getTab("AlignCommandsKShooter");
    kShooterEntry = tuningTab.add("Kshooter", 29.5).withWidget("NumberSlider").getEntry();
  }

  public static double getKShooter() {
    return kShooterEntry.getDouble(1);
  }

  private static GenericEntry inputsTemporalDisplacementEntry;
  private static GenericEntry v_linearTemporalDisplacementEntry;
  private static GenericEntry v_sTemporalDisplacementEntry;
  private static GenericEntry omegaTemporalDisplacementEntry;

  public static void initializeTemporalDisplacements() {
    ShuffleboardTab tuningTab = Shuffleboard.getTab("TemporalDisplacements");
    inputsTemporalDisplacementEntry =
        tuningTab.add("input lag(ms)", 100).withWidget("NumberSlider").getEntry();
    v_linearTemporalDisplacementEntry =
        tuningTab.add("v_linear(ms)", 80).withWidget("NumberSlider").getEntry();
    v_sTemporalDisplacementEntry =
        tuningTab.add("v_s(ms)", 80).withWidget("NumberSlider").getEntry();
    omegaTemporalDisplacementEntry =
        tuningTab.add("omega(ms)", 140).withWidget("NumberSlider").getEntry();
  }

  public static int getNumLagInputs() {
    return (int)
        Math.round(inputsTemporalDisplacementEntry.getDouble(100) / deltaT.in(Milliseconds));
  }

  public static int getVLinearDisplacement() {
    return (int)
        Math.round(v_linearTemporalDisplacementEntry.getDouble(80) / deltaT.in(Milliseconds));
  }

  public static int getShooterVelDisplacement() {
    return (int) Math.round(v_sTemporalDisplacementEntry.getDouble(80) / deltaT.in(Milliseconds));
  }

  public static int getOmegaDisplacement() {
    return (int)
        Math.round(omegaTemporalDisplacementEntry.getDouble(140) / deltaT.in(Milliseconds));
  }

  private static GenericEntry maxSpeedEntry;

  public static void initializeLimits() {
    ShuffleboardTab tuningTab = Shuffleboard.getTab("MoveShootLimits");
    maxSpeedEntry = tuningTab.add("Max Speed", 2.0).withWidget("NumberSlider").getEntry();
  }

  public static LinearVelocity getMaxSpeed() {
    return MetersPerSecond.of(maxSpeedEntry.getDouble(2.0));
  }

  public static boolean isFlipped =
      DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

  private static Translation3d hubLocation =
      new Translation3d(0, 0, (2.03 + 1.52) / 2); // lunar converter 152cm bottom - 203cm top
  private static final Translation3d shooterOffset =
      new Translation3d(-0.5, 0, Inches.of(16.081505).in(Meters));
  private static final Angle shooterAltitude = Degrees.of(50);

  private static AngularVelocity lastOmega = RadiansPerSecond.of(0);

  private static double deadzone(double input, double zone) {
    if (Math.abs(input) < zone) {
      return 0.0;
    } else {
      return input;
    }
  }

  public static Command resetDisplacement(Drive drive) {
    try {
      return Commands.runOnce(
          () -> {
            Translation2d fieldPos = drive.getPose().getTranslation();
            hubLocation =
                new Translation3d(
                    fieldPos.getX() + getDisplacementX(),
                    fieldPos.getY() + getDisplacementY(),
                    getDisplacementZ());
          },
          drive);
    } catch (Exception e) {
      DriverStation.reportError("Failed to create resetDisplacement command", e.getStackTrace());
      return Commands
          .none(); // catches exception in command creation during boot, prevents BOOT LOOP
    }
  }

  public static Translation3d calcShooterPos(Pose2d pose) {
    Translation3d currentShooter =
        shooterOffset.rotateBy(
            new Rotation3d(
                Radians.of(0), Radians.of(0), Radians.of(pose.getRotation().getRadians())));
    return currentShooter.plus(new Translation3d(pose.getX(), pose.getY(), 0));
  }

  public static Translation3d calcTargetDisplacement(double xPos, double yPos) {
    return hubLocation.minus(new Translation3d(xPos, yPos, shooterOffset.getZ()));
  }

  static double integrateTrapezoid(LinkedList<Double> values, double dt, int t) {
    double currentX = 0;
    for (int i = 0; i < t; i++) {
      double v_current = values.get(i);
      double v_next = values.get(i + 1);

      // Average velocity for this step * time
      currentX += ((v_current + v_next) / 2.0) * dt;
    }
    return currentX;
  }

  static double integrate(LinkedList<Double> valuesPast, double dt, int t) {
    double currentX = 0;
    for (int i = 0; (i < t) && (i < valuesPast.size()); i++) {
      // velocity for this step * time
      currentX += valuesPast.get(i) * dt;
    }
    return currentX;
  }

  static double integrate(
      LinkedList<Double> valuesPast, LinkedList<Double> valuesFuture, double dt, int t) {
    double currentX = 0;
    for (int i = 0; (i < t) && (i < valuesPast.size()); i++) {
      // velocity for this step * time
      currentX += valuesPast.get(i) * dt;
    }
    for (int i = 0; (i < t - valuesPast.size()) && (i < valuesFuture.size()); i++) {
      currentX += valuesFuture.get(i) * dt;
    }
    return currentX;
  }

  static double getPastFuture(
      LinkedList<Double> valuesPast, LinkedList<Double> valuesFuture, int t) {
    if (t < valuesPast.size()) return valuesPast.get(t);
    else return valuesFuture.get(t - valuesPast.size());
  }

  static ProfiledPIDController azimuthMovingPidTrap;

  public static class MoveShootCommand extends Command {
    private final Drive drive;
    private final Shooter shooter;
    DoubleSupplier inputX;
    DoubleSupplier inputY;
    private ExecutorService darthVadersStarDestroyer;

    public MoveShootCommand(
        Drive drive, Shooter shooter, DoubleSupplier inputX, DoubleSupplier inputY) {
      this.drive = drive;
      this.shooter = shooter;
      this.inputX = inputX;
      this.inputY = inputY;
      addRequirements(drive);
      addRequirements(shooter);
    }

    private int numLagInputs = getNumLagInputs();
    private int numAdvanceVel = getVLinearDisplacement();
    private int numAdvanceOmega = getOmegaDisplacement();
    private int numAdvanceShooter = getShooterVelDisplacement();

    LinkedList<Double> velXinputs = new LinkedList<>();
    LinkedList<Double> velYinputs = new LinkedList<>();

    LinkedList<Double> appliedVelX = new LinkedList<>();
    LinkedList<Double> appliedVelY = new LinkedList<>();

    @Override
    public void initialize() {
      nodesJNI.configureParameters(0.75, 0.0072, 0.069, 1.225);
      darthVadersStarDestroyer = Executors.newSingleThreadExecutor();

      numLagInputs = getNumLagInputs();
      numAdvanceVel = getVLinearDisplacement();
      numAdvanceOmega = getOmegaDisplacement();
      numAdvanceShooter = getShooterVelDisplacement();

      appliedVelX.clear();
      appliedVelY.clear();
      for (int a = 0; a < numAdvanceVel; a++) {
        appliedVelX.addLast(0.0);
        appliedVelY.addLast(0.0);
      }
      velXinputs.clear();
      velYinputs.clear();
      for (int a = 0; a < numLagInputs; a++) {
        velXinputs.addLast(0.0);
        velYinputs.addLast(0.0);
      }

      azimuthMovingPidTrap =
          new ProfiledPIDController(
              getKp(), getKi(), getKd(), new TrapezoidProfile.Constraints(3, 13));
      azimuthMovingPidTrap.enableContinuousInput(-Math.PI, Math.PI);
      azimuthMovingPidTrap.setTolerance(0.001, 0.01);
      azimuthMovingPidTrap.reset(
          new TrapezoidProfile.State(
              drive.getRotation().getRadians(), drive.getChassisSpeeds().omegaRadiansPerSecond));
    }

    @Override
    public void execute() {
      // shape inputs and add to lag queue
      Pair<LinearVelocity, LinearVelocity> vIn = clampInputs(inputX, inputY, getMaxSpeed());
      Pair<LinearVelocity, LinearVelocity> vLimited =
          limitVelocityByAccel(
              vIn,
              MetersPerSecond.of(velXinputs.peekLast()),
              MetersPerSecond.of(velYinputs.peekLast()));
      velXinputs.pollFirst();
      velYinputs.pollFirst();
      velXinputs.addLast(vLimited.getFirst().in(MetersPerSecond));
      velYinputs.addLast(vLimited.getSecond().in(MetersPerSecond));

      // run calculations
      Future<?> shooterFuture = darthVadersStarDestroyer.submit(this::runVShooter);
      double omega = computeOmega();

      LinearVelocity velX = MetersPerSecond.of(velXinputs.get(0));
      LinearVelocity velY = MetersPerSecond.of(velYinputs.get(0));
      var speedLinear =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              velX, velY, RadiansPerSecond.of(omega), drive.getRotation());
      var speedRotation = new ChassisSpeeds(0, -omega * shooterOffset.getX(), omega);

      drive.runVelocityDangerous(speedLinear.plus(speedRotation));
      try {
        shooterFuture.get();
      } catch (InterruptedException | ExecutionException e) {
        e.printStackTrace();
      }

      // record applied inputs for next iteration and logging
      appliedVelX.addLast(velX.in(MetersPerSecond));
      appliedVelY.addLast(velY.in(MetersPerSecond));
      appliedVelX.pollFirst();
      appliedVelY.pollFirst();
      Logger.recordOutput("moveShootCommand/Applied/velX", velX.in(MetersPerSecond));
      Logger.recordOutput("moveShootCommand/Applied/velY", velY.in(MetersPerSecond));

      // log real state and errors
      logRealPlusErrors();
      Threads.setCurrentThreadPriority(true, 10);
    }

    @Override
    public void end(boolean interrupted) {
      if (darthVadersStarDestroyer != null) {
        darthVadersStarDestroyer.shutdownNow();
      }
      drive.stop();
      shooter.setShooterVelocity(RadiansPerSecond.of(0));
    }

    public void runVShooter() {
      var shooterPos = calcShooterPos(drive.getPose());
      double xPos0 =
          shooterPos.getX()
              + integrate(appliedVelX, velXinputs, deltaT.in(Seconds), numAdvanceShooter);
      double yPos0 =
          shooterPos.getY()
              + integrate(appliedVelY, velYinputs, deltaT.in(Seconds), numAdvanceShooter);
      var target0 = calcTargetDisplacement(xPos0, yPos0);
      double velX0 = getPastFuture(appliedVelX, velXinputs, numAdvanceShooter);
      double velY0 = getPastFuture(appliedVelY, velYinputs, numAdvanceShooter);
      double xPos1 = xPos0 + velX0 * deltaT.in(Seconds);
      double yPos1 = yPos0 + velY0 * deltaT.in(Seconds);
      var target1 = calcTargetDisplacement(xPos1, yPos1);
      double velX1 = getPastFuture(appliedVelX, velXinputs, numAdvanceShooter + 1);
      double velY1 = getPastFuture(appliedVelY, velYinputs, numAdvanceShooter + 1);
      double startSolve = HALUtil.getFPGATime();
      TrajectorySolution solution0 =
          nodesJNI.newtonRhapsonSolveAirResistance(
              velX0,
              velY0,
              target0.getX(),
              target0.getY(),
              target0.getZ(),
              shooterAltitude.in(Radians));
      TrajectorySolution solution1 =
          nodesJNI.newtonRhapsonSolveAirResistance(
              velX1,
              velY1,
              target1.getX(),
              target1.getY(),
              target1.getZ(),
              shooterAltitude.in(Radians));
      double endSolve = HALUtil.getFPGATime();
      Logger.recordOutput(
          "moveShootCommand/Calculated/solveTimeMS", (endSolve - startSolve) / 1000.0);

      AngularVelocity shooterVel0 = RadiansPerSecond.of(solution0.shooterVel * getKShooter());
      AngularVelocity shooterVel1 = RadiansPerSecond.of(solution1.shooterVel * getKShooter());

      shooter.setShooterVelocityAndNext(shooterVel1, shooterVel0);
      Logger.recordOutput("moveShootCommand/Applied/shooterVel", shooterVel0);
    }

    public double computeOmega() {
      var shooterPos = calcShooterPos(drive.getPose());
      double xPos0 =
          shooterPos.getX()
              + integrate(appliedVelX, velXinputs, deltaT.in(Seconds), numAdvanceOmega);
      double yPos0 =
          shooterPos.getY()
              + integrate(appliedVelY, velYinputs, deltaT.in(Seconds), numAdvanceOmega);
      var targetPos0 = calcTargetDisplacement(xPos0, yPos0);
      double velX0 = getPastFuture(appliedVelX, velXinputs, numAdvanceOmega);
      double velY0 = getPastFuture(appliedVelY, velYinputs, numAdvanceOmega);

      double xPos1 = xPos0 + velX0 * deltaT.in(Seconds);
      double yPos1 = yPos0 + velY0 * deltaT.in(Seconds);
      var targetPos1 = calcTargetDisplacement(xPos1, yPos1);
      double velX1 = getPastFuture(appliedVelX, velXinputs, numAdvanceOmega + 1);
      double velY1 = getPastFuture(appliedVelY, velYinputs, numAdvanceOmega + 1);

      TrajectorySolution solution0 =
          nodesJNI.newtonRhapsonSolveAirResistance(
              velX0,
              velY0,
              targetPos0.getX(),
              targetPos0.getY(),
              targetPos0.getZ(),
              shooterAltitude.in(Radians));

      TrajectorySolution solution1 =
          nodesJNI.newtonRhapsonSolveAirResistance(
              velX1,
              velY1,
              targetPos1.getX(),
              targetPos1.getY(),
              targetPos1.getZ(),
              shooterAltitude.in(Radians));

      double theta0 = (solution0.azimuth + Math.PI) % (2 * Math.PI) - Math.PI;
      double theta1 = (solution1.azimuth + Math.PI) % (2 * Math.PI) - Math.PI;
      Logger.recordOutput("computeOmega/theta0", theta0);
      Logger.recordOutput("computeOmega/theta1", theta1);
      double omega1 = ((theta1 - theta0 + Math.PI) % (2 * Math.PI) - Math.PI) / deltaT.in(Seconds);

      double pidOmegaTrap =
          azimuthMovingPidTrap.calculate(
              drive.getRotation().getRadians(), new TrapezoidProfile.State(theta0, omega1));
      Logger.recordOutput("computeOmega/omega1", omega1);
      Logger.recordOutput("moveShootCommand/Applied/omega", pidOmegaTrap);
      return pidOmegaTrap;
    }

    @Override
    public boolean isFinished() {
      // Define the condition for the command to finish
      return false; // For a command that runs continuously until interrupted
    }

    public void logRealPlusErrors() {
      ChassisSpeeds realVel =
          ChassisSpeeds.fromRobotRelativeSpeeds(
              drive.getChassisSpeeds(),
              isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());
      AngularVelocity shooterVel = shooter.inputs1.shooterVelocityRadPerSec;
      logReal(realVel, shooterVel);
      logErrors(realVel, shooterVel);
    }

    public void logReal(ChassisSpeeds realVel, AngularVelocity shooterVel) {

      /*
       * record real state
       *
       */
      Logger.recordOutput("moveShootCommand/Real/posX", drive.getPose().getMeasureX());
      Logger.recordOutput("moveShootCommand/Real/posY", drive.getPose().getMeasureY());

      Logger.recordOutput(
          "moveShootCommand/Real/vX", MetersPerSecond.of(realVel.vxMetersPerSecond));
      Logger.recordOutput(
          "moveShootCommand/Real/vY", MetersPerSecond.of(realVel.vyMetersPerSecond));
      Logger.recordOutput(
          "moveShootCommand/Real/omega",
          RadiansPerSecond.of(drive.getChassisSpeeds().omegaRadiansPerSecond));
      Logger.recordOutput("moveShootCommand/Real/shooterVel", shooterVel.in(RadiansPerSecond));
      Logger.recordOutput("moveShootCommand/Real/theta", drive.getPose().getRotation());
    }

    public void logErrors(ChassisSpeeds realVel, AngularVelocity shooterVel) {
      var shooterPos = calcShooterPos(drive.getPose());
      Translation3d target = calcTargetDisplacement(shooterPos.getX(), shooterPos.getY());
      Logger.recordOutput("eSim/shooterVel", shooterVel.in(RadiansPerSecond) / getKShooter());
      var idealNow =
          nodesJNI.newtonRhapsonSolveAirResistance(
              realVel.vxMetersPerSecond,
              realVel.vyMetersPerSecond,
              target.getX(),
              target.getY(),
              target.getZ(),
              shooterAltitude.in(Radians));

      Logger.recordOutput("moveShootCommand/RealCalc/theta", idealNow.azimuth);
      Logger.recordOutput(
          "moveShootCommand/RealCalc/shooterVel", idealNow.shooterVel * getKShooter());

      double error =
          nodesJNI.minDistTrajectory(
              shooterVel.in(RadiansPerSecond) / getKShooter(),
              drive.getRotation().getRadians(),
              realVel.vxMetersPerSecond,
              realVel.vyMetersPerSecond,
              target.getX(),
              target.getY(),
              target.getZ(),
              shooterAltitude.in(Radians));
      Logger.recordOutput("moveShootCommand/Real/error", error);
      double errorShooterVel =
          nodesJNI.minDistTrajectory(
              shooterVel.in(RadiansPerSecond) / getKShooter(),
              idealNow.azimuth,
              realVel.vxMetersPerSecond,
              realVel.vyMetersPerSecond,
              target.getX(),
              target.getY(),
              target.getZ(),
              shooterAltitude.in(Radians));
      Logger.recordOutput("moveShootCommand/Real/errorShooterVel", errorShooterVel);
      double errorTheta =
          nodesJNI.minDistTrajectory(
              idealNow.shooterVel,
              drive.getRotation().getRadians(),
              realVel.vxMetersPerSecond,
              realVel.vyMetersPerSecond,
              target.getX(),
              target.getY(),
              target.getZ(),
              shooterAltitude.in(Radians));
      Logger.recordOutput("moveShootCommand/Real/errorTheta", errorTheta);
    }
  }

  static LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(2);

  private static Pair<LinearVelocity, LinearVelocity> limitVelocityByAccel(
      Pair<LinearVelocity, LinearVelocity> velocity,
      LinearVelocity lastVelX,
      LinearVelocity lastVelY) {
    Pair<LinearVelocity, LinearVelocity> velDiff =
        new Pair<>(velocity.getFirst().minus(lastVelX), velocity.getSecond().minus(lastVelY));

    double velDiffMag =
        Math.hypot(velDiff.getFirst().in(MetersPerSecond), velDiff.getSecond().in(MetersPerSecond));

    if (velDiffMag < maxAcceleration.times(Milliseconds.of(20)).in(MetersPerSecond)) {
      return velocity;
    } // velDiffMag > maxAcceleration.times(Milliseconds.of(20))
    double newDiffMag = maxAcceleration.times(Milliseconds.of(20)).in(MetersPerSecond);
    LinearVelocity velDiffAdjustedX = velDiff.getFirst().times(newDiffMag / velDiffMag);
    LinearVelocity velDiffAdjustedY = velDiff.getSecond().times(newDiffMag / velDiffMag);
    LinearVelocity vX = lastVelX.plus(velDiffAdjustedX);
    LinearVelocity vY = lastVelY.plus(velDiffAdjustedY);
    return new Pair<>(vX, vY);
  }

  private static Pair<LinearVelocity, LinearVelocity> clampInputs(
      DoubleSupplier inputX, DoubleSupplier inputY, LinearVelocity maxSpeed) {
    double xIn = deadzone(inputX.getAsDouble(), 0.05);
    double yIn = deadzone(inputY.getAsDouble(), 0.05);

    LinearVelocity vX, vY;
    double vMag = Math.sqrt(xIn * xIn + yIn * yIn);
    if (vMag > 1) {
      vX = maxSpeed.times(xIn / vMag);
      vY = maxSpeed.times(yIn / vMag);
    } else {
      vX = maxSpeed.times(xIn);
      vY = maxSpeed.times(yIn);
    }
    Logger.recordOutput("AlignCommands/vXin", vX);
    Logger.recordOutput("AlignCommands/vYin", vY);
    return new Pair<>(vX, vY);
  }
}
