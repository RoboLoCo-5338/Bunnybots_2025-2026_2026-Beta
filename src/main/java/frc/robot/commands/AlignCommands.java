package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

  static final double azimuthKp = 5.0;
  static final double azimuthKd = 0.4;
  static PIDController azimuthStationaryPid = new PIDController(azimuthKp, 0, azimuthKd);

  public static Command testAlignStationary(Drive drive, Shooter shooter, DoubleSupplier kShooter) {
    try {
      return Commands.runOnce(
              () -> {
                azimuthStationaryPid = new PIDController(azimuthKp, 0, azimuthKd);
                azimuthStationaryPid.enableContinuousInput(-Math.PI, Math.PI);
              })
          .andThen(
              Commands.runEnd(
                  () -> {
                    Translation2d fieldPos = drive.getPose().getTranslation();
                    Translation3d targetDisplacement =
                        hubLocation.minus(
                            shooterOffset.plus(
                                new Translation3d(fieldPos.getX(), fieldPos.getY(), 0)));
                    Logger.recordOutput("AlignStationary/targetDisplacement", targetDisplacement);
                    FixedTrajectorySolution solution =
                        ProjectileTrajectoryUtils.calcFiringSolution(
                            MetersPerSecond.of(0),
                            MetersPerSecond.of(0),
                            targetDisplacement,
                            shooterAltitude);
                    AngularVelocity shooterAngularVelocity =
                        RadiansPerSecond.of(
                            solution.shooterVelocity.in(MetersPerSecond) * kShooter.getAsDouble());

                    Logger.recordOutput(
                        "AlignStationary/ShooterRadiansPerSecond",
                        shooterAngularVelocity.in(RadiansPerSecond));
                    shooter.setShooterVelocity(shooterAngularVelocity);

                    Angle azimuthDiff =
                        Radians.of(
                            deadzone(
                                (solution.azimuth.in(Radians)
                                    - drive.getPose().getRotation().getRadians()),
                                0.001));
                    if (azimuthDiff.in(Degrees) > 180) {
                      azimuthDiff = Degrees.of(azimuthDiff.in(Degrees) - 360);
                    } else if (azimuthDiff.in(Degrees) < -180) {
                      azimuthDiff = Degrees.of(azimuthDiff.in(Degrees) + 360);
                    }

                    AngularVelocity omega = azimuthDiff.div(Seconds.of(0.1));
                    if (Math.abs(omega.in(RadiansPerSecond))
                        > drive.getMaxAngularSpeedRadPerSec()) {
                      omega =
                          RadiansPerSecond.of(
                              Math.copySign(
                                  drive.getMaxAngularSpeedRadPerSec(), omega.in(RadiansPerSecond)));
                    }
                    // omega =
                    //     RadiansPerSecond.of(
                    //         azimuthStationaryPid.calculate(
                    //             drive.getRotation().getRadians(), solution.azimuth.in(Radians)));
                    ChassisSpeeds speeds =
                        new ChassisSpeeds(MetersPerSecond.of(0), MetersPerSecond.of(0), omega);
                    drive.runVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds,
                            isFlipped
                                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                : drive.getRotation()));

                    Logger.recordOutput("AlignStationary/AngularSpeed", omega);
                    lastOmega = omega;

                    Logger.recordOutput("AlignStationary/azimuthDiff", azimuthDiff);

                    Distance error =
                        ProjectileTrajectoryUtils.minDistTrajectory(
                            MetersPerSecond.of(0),
                            MetersPerSecond.of(0),
                            targetDisplacement,
                            shooterAltitude,
                            Radians.of(drive.getPose().getRotation().getRadians()),
                            solution.shooterVelocity);

                    Logger.recordOutput("AlignStationary/minDistTrajectory", error);
                    Logger.recordOutput(
                        "AlignStationary/actualShooterRadPerSecond",
                        shooter.inputs1.shooterVelocityRadPerSec);
                  },
                  () -> {
                    drive.stop();
                    shooter.setShooterVelocity(RadiansPerSecond.of(0));
                  },
                  drive,
                  shooter));

    } catch (Exception e) {
      DriverStation.reportError("Failed to create AlignStationary command", e.getStackTrace());
      return Commands
          .none(); // catches exception in command creation during boot, prevents BOOT LOOP
    }
  }

  public static double startAlignMovingTime = 0.0;
  static Angle setAzimuth = Degrees.of(0);

  static Distance setX = Meters.of(0);
  static Distance setY = Meters.of(0);

  public static Command DONOTUSE_alignMoving(
      Drive drive,
      Shooter shooter,
      DoubleSupplier kShooter,
      LinearVelocity maxSpeed,
      DoubleSupplier inputX,
      DoubleSupplier inputY) {
    try {
      return Commands.runOnce(
              () -> {
                // azimuthMovingPid = new PIDController(azimuthKp, 0, azimuthKd);
                startAlignMovingTime = HALUtil.getFPGATime();
                Distance xPos = drive.getPose().getMeasureX();
                Distance yPos = drive.getPose().getMeasureY();
                setX = xPos;
                setY = yPos;
              })
          .andThen(
              Commands.runEnd(
                  () -> {
                    LinearVelocity realVx =
                        MetersPerSecond.of(
                            ChassisSpeeds.fromRobotRelativeSpeeds(
                                    drive.getChassisSpeeds(),
                                    isFlipped
                                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                        : drive.getRotation())
                                .vxMetersPerSecond);
                    LinearVelocity realVy =
                        MetersPerSecond.of(
                            ChassisSpeeds.fromRobotRelativeSpeeds(
                                    drive.getChassisSpeeds(),
                                    isFlipped
                                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                        : drive.getRotation())
                                .vyMetersPerSecond);

                    Logger.recordOutput("AlignMoving/realVx", realVx);
                    Logger.recordOutput("AlignMoving/realVy", realVy);
                    Distance xPos = drive.getPose().getMeasureX();
                    Distance yPos = drive.getPose().getMeasureY();
                    Logger.recordOutput("AlignMoving/setX", setX);
                    Logger.recordOutput("AlignMoving/setY", setY);
                    Logger.recordOutput("AlignMoving/xPos", xPos);
                    Logger.recordOutput("AlignMoving/yPos", yPos);

                    // clamp inputs
                    Pair<LinearVelocity, LinearVelocity> vIn =
                        clampInputs(inputX, inputY, maxSpeed);
                    LinearVelocity vX = vIn.getFirst();
                    LinearVelocity vY = vIn.getFirst();

                    Distance diffX = setX.minus(xPos);
                    Distance diffY = setY.minus(yPos);
                    Logger.recordOutput("AlignMoving/diffX", diffX);
                    Logger.recordOutput("AlignMoving/diffY", diffY);

                    setX = xPos.plus(vX.times(Milliseconds.of(20)));
                    setY = yPos.plus(vY.times(Milliseconds.of(20)));

                    Translation3d targetDisplacement =
                        hubLocation.minus(
                            shooterOffset.plus(
                                new Translation3d(xPos.in(Meters), yPos.in(Meters), 0)));
                    ProjectileTrajectoryUtils.MovingTrajectorySolution solution =
                        ProjectileTrajectoryUtils.calcMovingFiringSolutionConstantVelocity(
                            realVx, realVy, targetDisplacement, shooterAltitude);

                    AngularVelocity shooterAngularVelocity =
                        RadiansPerSecond.of(
                            solution.shooterVelocity.in(MetersPerSecond) * kShooter.getAsDouble());
                    Logger.recordOutput(
                        "AlignMoving/ShooterRadiansPerSecond", shooterAngularVelocity);
                    shooter.setShooterVelocity(shooterAngularVelocity);

                    AngularVelocity omega = solution.omega;
                    Angle azimuthDiff =
                        Radians.of(
                            deadzone(
                                (solution.azimuth.in(Radians)
                                    - drive.getPose().getRotation().getRadians()),
                                0.001));
                    if (azimuthDiff.in(Degrees) > 180) {
                      azimuthDiff = Degrees.of(azimuthDiff.in(Degrees) - 360);
                    } else if (azimuthDiff.in(Degrees) < -180) {
                      azimuthDiff = Degrees.of(azimuthDiff.in(Degrees) + 360);
                    }

                    // omega =
                    //     omega.plus(
                    //         RadiansPerSecond.of(
                    //             azimuthMovingPid.calculate(
                    //                 drive.getRotation().getRadians(),
                    //                 solution.azimuth.in(Radians))));
                    omega = omega.plus(azimuthDiff.div(Seconds.of(0.1)));

                    if (Math.abs(omega.in(RadiansPerSecond))
                        > drive.getMaxAngularSpeedRadPerSec()) {
                      omega =
                          RadiansPerSecond.of(
                              Math.copySign(
                                  drive.getMaxAngularSpeedRadPerSec(), omega.in(RadiansPerSecond)));
                    }

                    ChassisSpeeds speeds =
                        new ChassisSpeeds(
                            vX.plus(diffX.div(Seconds.of(0.5))),
                            vY.plus(diffY.div(Seconds.of(0.5))),
                            RadiansPerSecond.of(0));
                    speeds =
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds,
                            (isFlipped
                                    ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                    : drive.getRotation())
                                .plus(new Rotation2d(omega.times(Milliseconds.of(10)))));

                    ChassisSpeeds adjustedSpeeds =
                        new ChassisSpeeds(
                            speeds.vxMetersPerSecond,
                            speeds.vyMetersPerSecond,
                            omega.in(RadiansPerSecond));

                    drive.runVelocityDangerous(adjustedSpeeds);

                    Distance error =
                        ProjectileTrajectoryUtils.minDistTrajectory(
                            realVx,
                            realVy,
                            hubLocation.minus(
                                shooterOffset.plus(
                                    new Translation3d(xPos.in(Meters), yPos.in(Meters), 0))),
                            shooterAltitude,
                            Radians.of(drive.getPose().getRotation().getRadians()),
                            solution.shooterVelocity);

                    Logger.recordOutput(
                        "AlignMoving/minDistTrajectory", Math.min(error.in(Meters), 1));
                    Logger.recordOutput("AlignMoving/azimuth", solution.azimuth);
                  },
                  () -> {
                    drive.stop();
                    shooter.setShooterVelocity(RadiansPerSecond.of(0));
                  }));
    } catch (Exception e) {
      DriverStation.reportError("testAlignMoving", e.getStackTrace());
      return Commands
          .none(); // catches exception in command creation during boot, prevents BOOT LOOP
    }
  }

  static PIDController azimuthMovingPid = new PIDController(azimuthKp, 0, azimuthKd);

  static Deque<Distance> posXinputs = new LinkedList<>();
  static Deque<Distance> posYinputs = new LinkedList<>();
  static Iterator<Distance> posXit;
  static Iterator<Distance> posYit;
  static Deque<LinearVelocity> velXinputs = new LinkedList<>();
  static Deque<LinearVelocity> velYinputs = new LinkedList<>();
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
  private static int numAdvanceVel = 4; // compensating for pid lag
  private static Time deltaT = Milliseconds.of(20);

  private static int numAdvanceOmega = 4; // compensating for pid lag
  private static int numAdvanceShooter = 6;

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
                startAlignMovingTime = HALUtil.getFPGATime();
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
                for (int i = 0; i < numLagInputs + 1; i++) {
                  posXinputs.offerLast(drive.getPose().getMeasureX());
                  posYinputs.offerLast(drive.getPose().getMeasureY());
                }
                for (int i = 0; i < numLagInputs + 1; i++) {
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

                for (int i = 0; i < numLagInputs - 1; i++) {
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
                    shooterVelCalc.offerLast(shooterAngularVelocity);
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
                    shooter.setShooterVelocity(shooterVel.times(0.1647809660 * 0.9890681004));
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
    Logger.recordOutput("testInputShape/Real/shooterVel", shooter.inputs1.shooterVelocityRadPerSec);
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
            MetersPerSecond.of(
                shooter.inputs1.shooterVelocityRadPerSec.in(RadiansPerSecond)
                    / kShooter.getAsDouble()));

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
