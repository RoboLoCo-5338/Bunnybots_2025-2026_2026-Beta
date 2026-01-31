package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ProjectileTrajectoryUtils;
import frc.robot.util.ProjectileTrajectoryUtils.AirResistanceSolver.TrajectorySolution;
import frc.robot.util.ProjectileTrajectoryUtils.FixedTrajectorySolution;
import frc.robot.util.ProjectileTrajectoryUtils.MovingTrajectorySolution;
import java.util.Deque;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AlignCommands {
  public static boolean isFlipped =
      DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

  private AlignCommands() {}

  private static Translation3d hubLocation =
      new Translation3d(0, 0, (2.03 + 1.52) / 2); // lunar converter 152cm bottom - 203cm top
  private static final Translation3d shooterOffset =
      new Translation3d(-Inches.of(0.1956095).in(Meters), 0, Inches.of(16.081505).in(Meters));
  private static final Angle shooterAltitude = Degrees.of(50);

  private static AngularVelocity lastOmega = RadiansPerSecond.of(0);

  private static double deadzone(double input, double zone) {
    if (Math.abs(input) < zone) {
      return 0.0;
    } else {
      return input;
    }
  }

  public static Command resetDisplacement(
      Drive drive,
      DoubleSupplier displacementX,
      DoubleSupplier displacementY,
      DoubleSupplier displacementZ) {
    try {
      return Commands.runOnce(
          () -> {
            Translation2d fieldPos = drive.getPose().getTranslation();
            hubLocation =
                new Translation3d(
                    fieldPos.getX() + displacementX.getAsDouble(),
                    fieldPos.getY() + displacementY.getAsDouble(),
                    displacementZ.getAsDouble());
          },
          drive);
    } catch (Exception e) {
      DriverStation.reportError("Failed to create resetDisplacement command", e.getStackTrace());
      return Commands
          .none(); // catches exception in command creation during boot, prevents BOOT LOOP
    }
  }

  public static Translation3d calcTargetDisplacement(double xPos, double yPos) {
    return hubLocation.minus(shooterOffset.plus(new Translation3d(xPos, yPos, 0)));
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

  static double integrate(LinkedList<Double> values, double dt, int t) {
    double currentX = 0;
    for (int i = 0; i < t; i++) {
      // velocity for this step * time
      currentX += values.get(i) * dt;
    }
    return currentX;
  }

  static Deque<LinearVelocity> velXinputs = new LinkedList<>();
  static Deque<LinearVelocity> velYinputs = new LinkedList<>();

  static LinkedList<Double> appliedVelX = new LinkedList<>();
  static LinkedList<Double> appliedVelY = new LinkedList<>();
  static LinkedList<Double> appliedOmega = new LinkedList<>();

  public static Command moveShootCommand(
      Drive drive,
      Shooter shooter,
      DoubleSupplier kShooter,
      LinearVelocity maxSpeed,
      DoubleSupplier inputX,
      DoubleSupplier inputY) {
    try {
      return Commands.runOnce(
              () -> {
                appliedVelX.clear();
                appliedVelY.clear();
                appliedOmega.clear();
                for (int a = 0; a < 13; a++) {
                  appliedVelX.addLast(0.0);
                  appliedVelY.addLast(0.0);
                }
                for (int a = 0; a < 13; a++) {
                  appliedOmega.addLast(0.0);
                }
                velXinputs.clear();
                velYinputs.clear();
                for (int a = 0; a < 1; a++) {
                  velXinputs.addLast(MetersPerSecond.of(0.0));
                  velYinputs.addLast(MetersPerSecond.of(0.0));
                }
              })
          .andThen(
              Commands.runEnd(
                  () -> {
                    double xPosFuture =
                        drive.getPose().getX()
                            + integrate(appliedVelX, deltaT.in(Seconds), numAdvanceVel);
                    double yPosFuture =
                        drive.getPose().getY()
                            + integrate(appliedVelY, deltaT.in(Seconds), numAdvanceVel);
                    double thetaFuture =
                        drive.getRotation().getRadians()
                            + integrate(appliedOmega, deltaT.in(Seconds), numAdvanceOmega);
                    Logger.recordOutput("moveShootCommand/Predicted/xPosFuture", xPosFuture);
                    Logger.recordOutput("moveShootCommand/Predicted/yPosFuture", yPosFuture);
                    Logger.recordOutput("moveShootCommand/Predicted/thetaFuture", thetaFuture);
                    Pair<LinearVelocity, LinearVelocity> vIn =
                        clampInputs(inputX, inputY, maxSpeed);
                    Pair<LinearVelocity, LinearVelocity> vLimited =
                        limitVelocityByAccel(
                            vIn,
                            MetersPerSecond.of(appliedVelX.peekFirst()),
                            MetersPerSecond.of(appliedVelY.peekFirst()));
                    velXinputs.addLast(vLimited.getFirst());
                    velYinputs.addLast(vLimited.getSecond());
                    LinearVelocity velX = velXinputs.pollFirst();
                    LinearVelocity velY = velYinputs.pollFirst();

                    double startSolve = HALUtil.getFPGATime();
                    TrajectorySolution solution =
                        ProjectileTrajectoryUtils.AirResistanceSolver
                            .newtonRhapsonSolveAirResistance(
                                velX,
                                velY,
                                calcTargetDisplacement(xPosFuture, yPosFuture),
                                shooterAltitude);
                    double endSolve = HALUtil.getFPGATime();
                    Logger.recordOutput(
                        "moveShootCommand/Calculated/solveTimeMS",
                        (endSolve - startSolve) / 1000.0);
                    AngularVelocity shooterAngularVelocity =
                        RadiansPerSecond.of(
                            solution.shooterVelocity.in(MetersPerSecond) * kShooter.getAsDouble());
                    Angle theta =
                        Radians.of(
                            (solution.azimuth.in(Radians) + Math.PI) % (2 * Math.PI) - Math.PI);
                    Logger.recordOutput("moveShootCommand/Calculated/theta", theta);
                    Angle azimuthDiff = Radians.of((theta.in(Radians) - thetaFuture));
                    if (azimuthDiff.in(Radians) > Math.PI) {
                      azimuthDiff = Radians.of(azimuthDiff.in(Radians) - 2 * Math.PI);
                    } else if (azimuthDiff.in(Radians) < -Math.PI) {
                      azimuthDiff = Radians.of(azimuthDiff.in(Radians) + 2 * Math.PI);
                    }
                    Logger.recordOutput("moveShootCommand/Calculated/azimuthDiff", azimuthDiff);
                    double maxAngularAccel = 3.0;
                    double maxAngularVel = 3.0;
                    double omega = azimuthDiff.in(Radians) / deltaT.in(Seconds);
                    if (Math.abs(azimuthDiff.in(Radians)) > 0.005) {
                      double angularAccel =
                          Math.abs(
                              appliedOmega.peekFirst()
                                  * appliedOmega.peekFirst()
                                  / (2 * azimuthDiff.in(Radians)));
                      if (angularAccel > 2.0) {
                        angularAccel = Math.min(angularAccel, maxAngularAccel);
                        omega =
                            appliedOmega.peekFirst()
                                - Math.copySign(angularAccel, azimuthDiff.in(Radians))
                                    * deltaT.in(Seconds);
                      } else {
                        omega =
                            Math.copySign(maxAngularAccel, azimuthDiff.in(Radians))
                                    * deltaT.in(Seconds)
                                + appliedOmega.peekFirst();
                      }
                      omega = Math.max(Math.min(omega, maxAngularVel), -maxAngularVel);
                    } else {
                      omega = 0.0;
                    }

                    drive.runVelocityDangerous(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            velX, velY, RadiansPerSecond.of(omega), new Rotation2d(thetaFuture)));
                    appliedVelX.addFirst(velX.in(MetersPerSecond));
                    appliedVelY.addFirst(velY.in(MetersPerSecond));
                    appliedOmega.addFirst(omega);
                    Logger.recordOutput("moveShootCommand/Applied/velX", velX.in(MetersPerSecond));
                    Logger.recordOutput("moveShootCommand/Applied/velY", velY.in(MetersPerSecond));
                    Logger.recordOutput("moveShootCommand/Applied/omega", omega);
                    appliedVelX.pollLast();
                    appliedVelY.pollLast();
                    appliedOmega.pollLast();
                    AngularVelocity shooterVel =
                        RadiansPerSecond.of(
                            solution.shooterVelocity.in(MetersPerSecond) * kShooter.getAsDouble());
                    if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
                      if (shooterVel.in(RadiansPerSecond) > 0)
                        shooter.setShooterVelocityAndAcceleration(
                            shooterVel.times(0.1647809660 * 0.9890681004),
                            lastShooterVel.times(0.1647809660 * 0.9890681004));
                    } else if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
                      if (shooterVel.in(RadiansPerSecond) > 0
                          && shooterVel.in(RadiansPerSecond) < 500)
                        shooter.setShooterVelocityAndAcceleration(
                            shooterVel.times(1), lastShooterVel.times(1));
                    }
                    Logger.recordOutput(
                        "testInputShape/Applied/shooterVel", shooterVel.in(RadiansPerSecond));
                    logRealPlusError(drive, shooter, kShooter);
                    lastShooterVel = shooterVel;
                  },
                  () -> {
                    drive.stop();
                    shooter.setShooterVelocity(RadiansPerSecond.of(0));
                  }));

    } catch (Exception e) {
      DriverStation.reportError("testInputShape", e.getStackTrace());
      return Commands
          .none(); // catches exception in command creation during boot, prevents BOOT LOOP
    }
  }

  static AngularVelocity lastShooterVel = RadiansPerSecond.of(0);

  static PIDController azimuthMovingPid = new PIDController(5, 0, 1);
  static Deque<Distance> posXinputs = new LinkedList<>();
  static Deque<Distance> posYinputs = new LinkedList<>();
  static Iterator<Distance> posXit;
  static Iterator<Distance> posYit;
  static Iterator<LinearVelocity> velXit;
  static Iterator<LinearVelocity> velYit;
  static Deque<LinearAcceleration> accelXinputs = new LinkedList<>();
  static Deque<LinearAcceleration> accelYinputs = new LinkedList<>();
  static Iterator<LinearAcceleration> accelXit;
  static Iterator<LinearAcceleration> accelYit;
  static Deque<Angle> thetaCalc = new LinkedList<>();
  static Deque<AngularVelocity> omegaCalc = new LinkedList<>();
  static Deque<AngularVelocity> shooterVelCalc = new LinkedList<>();
  static Iterator<Angle> thetaIt;
  static Iterator<AngularVelocity> omegaIt;
  static Iterator<AngularVelocity> v_sIt;
  private static int numLagInputs = 10;
  private static int numAdvanceVel =
      Constants.CURRENT_MODE == Constants.Mode.SIM ? 6 : 4; // compensating for pid lag
  private static Time deltaT = Milliseconds.of(20);

  private static int numAdvanceOmega =
      Constants.CURRENT_MODE == Constants.Mode.SIM ? 6 : 4; // compensating for pid lag
  private static int numAdvanceShooter = 4;

  public static Command testInputShape(
      Drive drive,
      Shooter shooter,
      DoubleSupplier kShooter,
      LinearVelocity maxSpeed,
      DoubleSupplier inputX,
      DoubleSupplier inputY,
      DoubleSupplier kPtheta,
      DoubleSupplier kItheta,
      DoubleSupplier kDtheta) {
    try {
      return Commands.runOnce(
              () -> {
                azimuthMovingPid =
                    new PIDController(
                        kPtheta.getAsDouble(), kItheta.getAsDouble(), kDtheta.getAsDouble());
                azimuthMovingPid.enableContinuousInput(-Math.PI, Math.PI);
                azimuthMovingPid.setTolerance(0.01);
                FixedTrajectorySolution solution =
                    ProjectileTrajectoryUtils.calcFiringSolution(
                        MetersPerSecond.of(0),
                        MetersPerSecond.of(0),
                        hubLocation.minus(
                            shooterOffset.plus(
                                new Translation3d(
                                    drive.getPose().getX(), drive.getPose().getY(), 0))),
                        shooterAltitude);
                posXinputs.clear();
                posYinputs.clear();
                velXinputs.clear();
                velYinputs.clear();
                accelXinputs.clear();
                accelYinputs.clear();
                thetaCalc.clear();
                omegaCalc.clear();
                shooterVelCalc.clear();
                for (int i = 0; i < numLagInputs; i++) {
                  posXinputs.offerLast(drive.getPose().getMeasureX());
                  posYinputs.offerLast(drive.getPose().getMeasureY());
                }
                for (int i = 0; i < numLagInputs; i++) {
                  thetaCalc.offerLast(solution.azimuth);
                }
                for (int i = 0; i < numLagInputs; i++) {
                  shooterVelCalc.offerLast(
                      RadiansPerSecond.of(
                          solution.shooterVelocity.in(MetersPerSecond) * kShooter.getAsDouble()));
                }

                for (int i = 0; i < numLagInputs; i++) {
                  velXinputs.offerLast(MetersPerSecond.of(0));
                  velYinputs.offerLast(MetersPerSecond.of(0));
                }

                for (int i = 0; i < numLagInputs; i++) {
                  accelXinputs.offerLast(MetersPerSecondPerSecond.of(0));
                  accelYinputs.offerLast(MetersPerSecondPerSecond.of(0));
                }
                for (int i = 0; i < numLagInputs; i++) {
                  omegaCalc.offerLast(RadiansPerSecond.of(0));
                }
              })
          .andThen(
              Commands.runEnd(
                  () -> {

                    // calculation values:
                    // time-synchronized
                    Distance calcX, calcY;
                    LinearVelocity calcVx, calcVy;
                    LinearAcceleration calcAx, calcAy;

                    // process inputs
                    Pair<LinearVelocity, LinearVelocity> vIn =
                        clampInputs(inputX, inputY, maxSpeed);
                    Pair<LinearVelocity, LinearVelocity> vLimited =
                        limitVelocityByAccel(vIn, velXinputs.peekLast(), velYinputs.peekLast());
                    accelXinputs.offer(
                        (vLimited.getFirst().minus(velXinputs.peekLast())).div(deltaT));
                    accelYinputs.offer(
                        (vLimited.getSecond().minus(velYinputs.peekLast())).div(deltaT));
                    calcAx = accelXinputs.peekLast();
                    calcAy = accelYinputs.peekLast();
                    calcVx = velXinputs.peekLast();
                    calcVy = velYinputs.peekLast();
                    calcX = posXinputs.peekLast();
                    calcY = posYinputs.peekLast();
                    velXinputs.offerLast(vLimited.getFirst());
                    velYinputs.offerLast(vLimited.getSecond());
                    posXinputs.offerLast(
                        posXinputs
                            .peekLast()
                            .plus(calcVx.times(deltaT))
                            .plus(calcAx.times(deltaT).times(deltaT).times(0.5)));
                    posYinputs.offerLast(
                        posYinputs
                            .peekLast()
                            .plus(calcVy.times(deltaT))
                            .plus(calcAy.times(deltaT).times(deltaT).times(0.5)));
                    Logger.recordOutput("testInputShape/vInX", vIn.getFirst());
                    Logger.recordOutput("testInputShape/vInY", vIn.getSecond());
                    Logger.recordOutput("testInputShape/vLimitedX", vLimited.getFirst());
                    Logger.recordOutput("testInputShape/vLimitedY", vLimited.getSecond());

                    MovingTrajectorySolution solution =
                        ProjectileTrajectoryUtils.calcMovingFiringSolution(
                            calcVx,
                            calcVy,
                            calcAx,
                            calcAy,
                            hubLocation.minus(
                                shooterOffset.plus(
                                    new Translation3d(calcX.in(Meters), calcY.in(Meters), 0))),
                            shooterAltitude);

                    AngularVelocity shooterAngularVelocity =
                        RadiansPerSecond.of(
                            solution.shooterVelocity.in(MetersPerSecond) * kShooter.getAsDouble());
                    shooterVelCalc.offerLast(
                        RadiansPerSecond.of(vLimited.getFirst().in(MetersPerSecond) * 100));
                    thetaCalc.offerLast(solution.azimuth);
                    omegaCalc.offerLast(solution.omega);
                    Logger.recordOutput("testInputShape/Calculated/theta", solution.azimuth);
                    Logger.recordOutput("testInputShape/Calculated/omega", solution.omega);
                    Logger.recordOutput(
                        "testInputShape/Calculated/shooterVel", shooterAngularVelocity);

                    Logger.recordOutput("testInputShape/Calculated/accelX", calcAx);
                    Logger.recordOutput("testInputShape/Calculated/accelY", calcAy);
                    Logger.recordOutput("testInputShape/Calculated/velX", calcVx);
                    Logger.recordOutput("testInputShape/Calculated/velY", calcVy);
                    Logger.recordOutput("testInputShape/Calculated/posX", calcX);
                    Logger.recordOutput("testInputShape/Calculated/posY", calcY);

                    /*
                     *
                     *
                     * apply lagged inputs
                     *
                     *
                     */
                    posXit = posXinputs.iterator();
                    posYit = posYinputs.iterator();
                    thetaIt = thetaCalc.iterator();
                    v_sIt = shooterVelCalc.iterator();
                    for (int i = 0; i < numAdvanceShooter; i++) v_sIt.next();

                    velXit = velXinputs.iterator();
                    velYit = velYinputs.iterator();
                    for (int i = 0; i < numAdvanceVel; i++) {
                      velXit.next();
                      velYit.next();
                    }
                    accelXit = accelXinputs.iterator();
                    accelYit = accelYinputs.iterator();
                    omegaIt = omegaCalc.iterator();
                    for (int i = 0; i < numAdvanceOmega; i++) omegaIt.next();

                    Distance x = posXit.next();
                    Distance y = posYit.next();
                    LinearVelocity vX = velXit.next();
                    LinearVelocity vY = velYit.next();
                    LinearAcceleration aX = accelXit.next();
                    LinearAcceleration aY = accelYit.next();
                    Logger.recordOutput("testInputShape/Applied/posX", x);
                    Logger.recordOutput("testInputShape/Applied/posY", y);
                    Logger.recordOutput("testInputShape/Applied/vX", vX);
                    Logger.recordOutput("testInputShape/Applied/vY", vY);
                    if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
                      vX = vX.times(0.9194105329);
                      vY = vY.times(0.9194105329);
                    } else if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
                      vX = vX.times(0.9963632740);
                      vY = vY.times(0.9963632740);
                    }
                    Logger.recordOutput("testInputShape/Applied/aX", aX);
                    Logger.recordOutput("testInputShape/Applied/aY", aY);
                    // Distance xErr = x.minus(drive.getPose().getMeasureX());
                    // Distance yErr = y.minus(drive.getPose().getMeasureY());
                    // vX = vX.plus(xErr.div(Milliseconds.of(500)).times(0.1));
                    // vY = vY.plus(yErr.div(Milliseconds.of(500)).times(0.1));
                    Angle theta = thetaIt.next();
                    AngularVelocity omega = omegaIt.next();
                    Angle azimuthDiff =
                        Radians.of(
                            deadzone(
                                (theta.in(Radians) - drive.getPose().getRotation().getRadians()),
                                0.001));
                    if (azimuthDiff.in(Degrees) > 180) {
                      azimuthDiff = Degrees.of(azimuthDiff.in(Degrees) - 360);
                    } else if (azimuthDiff.in(Degrees) < -180) {
                      azimuthDiff = Degrees.of(azimuthDiff.in(Degrees) + 360);
                    }
                    // omega = omega.plus(azimuthDiff.div(Seconds.of(0.1)));
                    AngularVelocity pidOmega =
                        RadiansPerSecond.of(
                            azimuthMovingPid.calculate(
                                drive.getRotation().getRadians(), theta.in(Radians)));
                    if (azimuthMovingPid.atSetpoint()) {
                      pidOmega = RadiansPerSecond.of(0);
                    }
                    // omega = RadiansPerSecond.of(0);
                    // pidOmega = RadiansPerSecond.of(0);

                    ChassisSpeeds speeds = new ChassisSpeeds(vX, vY, omega.plus(pidOmega));
                    speeds =
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds,
                            isFlipped
                                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                : drive.getRotation());
                    drive.runVelocity(speeds);
                    AngularVelocity shooterVel = v_sIt.next();
                    if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
                      if (shooterVel.in(RadiansPerSecond) > 0)
                        shooter.setShooterVelocity(shooterVel.times(0.1647809660 * 0.9890681004));
                    } else if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
                      if (shooterVel.in(RadiansPerSecond) > 0
                          && shooterVel.in(RadiansPerSecond) < 500)
                        shooter.setShooterVelocity(shooterVel.times(.1));
                    }
                    Logger.recordOutput("testInputShape/Applied/shooterVel", shooterVel);
                    Logger.recordOutput("testInputShape/Applied/omega", omega);
                    Logger.recordOutput("testInputShape/Applied/theta", theta);

                    logRealPlusError(drive, shooter, kShooter);

                    pollIdealDelayed();
                  },
                  () -> {
                    drive.stop();
                    shooter.setShooterVelocity(RadiansPerSecond.of(0));
                  }));

    } catch (Exception e) {
      DriverStation.reportError("testInputShape", e.getStackTrace());
      return Commands
          .none(); // catches exception in command creation during boot, prevents BOOT LOOP
    }
  }

  public static void logRealPlusError(Drive drive, Shooter shooter, DoubleSupplier kShooter) {

    /*
     * record real state
     *
     */
    Logger.recordOutput("testInputShape/Real/posX", drive.getPose().getMeasureX());
    Logger.recordOutput("testInputShape/Real/posY", drive.getPose().getMeasureY());

    ChassisSpeeds realVel =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drive.getChassisSpeeds(),
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());
    Logger.recordOutput("testInputShape/Real/vX", MetersPerSecond.of(realVel.vxMetersPerSecond));
    Logger.recordOutput("testInputShape/Real/vY", MetersPerSecond.of(realVel.vyMetersPerSecond));
    Logger.recordOutput(
        "testInputShape/Real/omega",
        RadiansPerSecond.of(drive.getChassisSpeeds().omegaRadiansPerSecond));
    AngularVelocity shooterVel = shooter.inputs1.shooterVelocityRadPerSec;
    if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
      // shooterVel = shooterVel.times(260 / 16.0 * 200.0 / 204);
    }
    Logger.recordOutput("testInputShape/Real/shooterVel", shooterVel.in(RadiansPerSecond));
    Logger.recordOutput("testInputShape/Real/theta", drive.getPose().getRotation());

    /*
     *
     * calculate error
     *
     *
     *
     */

    Distance error =
        ProjectileTrajectoryUtils.minDistTrajectory(
            MetersPerSecond.of(realVel.vxMetersPerSecond),
            MetersPerSecond.of(realVel.vyMetersPerSecond),
            hubLocation.minus(
                shooterOffset.plus(
                    new Translation3d(drive.getPose().getX(), drive.getPose().getY(), 0))),
            shooterAltitude,
            Radians.of(drive.getPose().getRotation().getRadians()),
            MetersPerSecond.of(shooterVel.in(RadiansPerSecond) / kShooter.getAsDouble()));

    Matrix<N2, N1> displacement =
        ProjectileTrajectoryUtils.AirResistanceSolver.f_airResistance(
            new Matrix<>(
                Nat.N2(),
                Nat.N1(),
                new double[] {
                  shooterVel.in(RadiansPerSecond) / kShooter.getAsDouble(),
                  drive.getPose().getRotation().getRadians()
                }),
            realVel.vxMetersPerSecond,
            realVel.vyMetersPerSecond,
            hubLocation.getZ() - shooterOffset.getZ(),
            shooterAltitude.in(Radians));
    Translation3d target = calcTargetDisplacement(drive.getPose().getX(), drive.getPose().getY());

    error =
        Meters.of(
            Math.hypot(
                target.getX() - displacement.get(0, 0), target.getY() - displacement.get(1, 0)));
    Logger.recordOutput("testInputShape/Real/minDistTrajectory", error);
  }

  public static void pollIdealDelayed() {
    Logger.recordOutput("testInputShape/Ideal/posX", posXinputs.pollFirst());
    Logger.recordOutput("testInputShape/Ideal/posY", posYinputs.pollFirst());
    Logger.recordOutput("testInputShape/Ideal/vX", velXinputs.pollFirst());
    Logger.recordOutput("testInputShape/Ideal/vY", velYinputs.pollFirst());
    Logger.recordOutput("testInputShape/Ideal/aX", accelXinputs.pollFirst());
    Logger.recordOutput("testInputShape/Ideal/aY", accelYinputs.pollFirst());
    Logger.recordOutput("testInputShape/Ideal/theta", thetaCalc.pollFirst());
    Logger.recordOutput("testInputShape/Ideal/omega", omegaCalc.pollFirst());
    Logger.recordOutput("testInputShape/Ideal/shooterVel", shooterVelCalc.pollFirst());
  }

  //   Torque maxTorque = NewtonMeters.of(10);
  //   Force maxForce = Newtons.of(10);
  static LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(1);

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
