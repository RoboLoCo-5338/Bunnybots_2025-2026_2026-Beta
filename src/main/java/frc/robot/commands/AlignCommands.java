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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
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

  public static Translation3d calcShooterPos(Pose2d pose) {
    Translation3d currentShooter =
        shooterOffset.rotateBy(
            new Rotation3d(
                Radians.of(0), Radians.of(0), Radians.of(pose.getRotation().getRadians())));
    return currentShooter.plus(new Translation3d(pose.getX(), pose.getY(), 0));
  }

  public static Translation3d calcTargetDisplacement(double xPos, double yPos) {
    return hubLocation.minus(new Translation3d(xPos, yPos, 0));
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

  static LinkedList<LinearVelocity> velXinputs = new LinkedList<>();
  static LinkedList<LinearVelocity> velYinputs = new LinkedList<>();

  static LinkedList<Double> appliedVelX = new LinkedList<>();
  static LinkedList<Double> appliedVelY = new LinkedList<>();
  static LinkedList<Double> appliedOmega = new LinkedList<>();

  double maxAngularAccel = 3.0;
  double maxAngularVel = 3.0;

  public static double computeOmega(Drive drive, Shooter shooter, DoubleSupplier kShooter) {

    var shooterPos = calcShooterPos(drive.getPose());
    double xPos0 = shooterPos.getX() + integrate(appliedVelX, deltaT.in(Seconds), numAdvanceOmega);
    double yPos0 = shooterPos.getY() + integrate(appliedVelY, deltaT.in(Seconds), numAdvanceOmega);
    double theta_1 =
        (drive.getRotation().getRadians()
                    + integrate(appliedOmega, deltaT.in(Seconds), numAdvanceOmega)
                    + Math.PI)
                % (2 * Math.PI)
            - Math.PI;

    LinearVelocity velX0 = velXinputs.get(0);
    LinearVelocity velY0 = velYinputs.get(0);
    var targetPos0 = calcTargetDisplacement(xPos0, yPos0);
    TrajectorySolution solution0 =
        nodesJNI.newtonRhapsonSolveAirResistance(
            velX0.in(MetersPerSecond),
            velY0.in(MetersPerSecond),
            targetPos0.getX(),
            targetPos0.getY(),
            targetPos0.getZ(),
            shooterAltitude.in(Radians));

    LinearVelocity velX1 = velXinputs.get(1);
    LinearVelocity velY1 = velYinputs.get(1);
    double xPos1 = xPos0 += velX0.times(deltaT).in(Meters);
    double yPos1 = yPos0 += velY0.times(deltaT).in(Meters);
    var targetPos1 = calcTargetDisplacement(xPos1, yPos1);
    TrajectorySolution solution1 =
        nodesJNI.newtonRhapsonSolveAirResistance(
            velX1.in(MetersPerSecond),
            velY1.in(MetersPerSecond),
            targetPos1.getX(),
            targetPos1.getY(),
            targetPos1.getZ(),
            shooterAltitude.in(Radians));

    double theta0 = (solution0.azimuth + Math.PI) % (2 * Math.PI) - Math.PI;
    double theta1 = (solution1.azimuth + Math.PI) % (2 * Math.PI) - Math.PI;
    Logger.recordOutput("computeOmega/theta_1", theta_1);
    Logger.recordOutput("computeOmega/theta0", theta0);
    Logger.recordOutput("computeOmega/theta1", theta1);

    double omega_1 = (appliedOmega.peekFirst() + Math.PI) % (2 * Math.PI) - Math.PI;
    // compute new omega, accounting for -Pi to Pi wraparound
    double omega0 = ((theta0 - theta_1 + Math.PI) % (2 * Math.PI) - Math.PI) / deltaT.in(Seconds);
    double omega1 = ((theta1 - theta0 + Math.PI) % (2 * Math.PI) - Math.PI) / deltaT.in(Seconds);

    double minAngularAccel = 6.0;
    double maxAngularAccel = 9.0;
    double maxAngularVel = 3.0;
    omega0 = Math.max(Math.min(omega0, maxAngularVel), -maxAngularVel);
    omega1 = Math.max(Math.min(omega1, maxAngularVel), -maxAngularVel);
    Logger.recordOutput("computeOmega/omega0", omega0);
    Logger.recordOutput("computeOmega/omega1", omega1);

    double alpha0 = (omega0 - omega_1) / deltaT.in(Seconds);
    double alpha1 = (omega1 - omega0) / deltaT.in(Seconds);
    Logger.recordOutput("computeOmega/alpha0", alpha0);
    Logger.recordOutput("computeOmega/alpha1", alpha1);

    double deltaTheta = ((theta0 - theta_1 + Math.PI) % (2 * Math.PI) - Math.PI);
    Logger.recordOutput("computeOmega/deltaTheta", deltaTheta);

    double pidOmegaTrap =
        azimuthMovingPidTrap.calculate(
            drive.getRotation().getRadians(), new TrapezoidProfile.State(theta0, omega1));
    // if (azimuthMovingPid.atSetpoint()) {
    //   pidOmega = RadiansPerSecond.of(0);
    // }
    return pidOmegaTrap;

    // if (deltaTheta < 0.2 && Math.abs(alpha0) < 16 && Math.abs(alpha1) < 16) {
    //   Logger.recordOutput("computeOmega/if", 0);
    //   return (omega0 + omega_1) / 2;
    // }

    // double alphaReq = (omega1 * omega1 - omega_1 * omega_1) / (2 * deltaTheta);
    // Logger.recordOutput("computeOmega/alphaReq", alphaReq);
    // if (Math.abs(alphaReq) > maxAngularAccel) {
    //   alphaReq = Math.copySign(maxAngularAccel, alphaReq);
    // }
    // if (Math.abs(alphaReq) > minAngularAccel) {
    //   Logger.recordOutput("computeOmega/if", 1);
    //   return omega_1 + alphaReq * deltaT.in(Seconds);
    // }
    // Logger.recordOutput("computeOmega/if", 2);
    // return omega_1 + Math.copySign(maxAngularAccel, deltaTheta) * deltaT.in(Seconds);
  }

  static ProfiledPIDController azimuthMovingPidTrap;

  public static Command moveShootCommand(
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
                nodesJNI.configureParameters(0.75, 0.0072, 0.069, 1.225);

                azimuthMovingPidTrap =
                    new ProfiledPIDController(
                        kPtheta.getAsDouble(),
                        kItheta.getAsDouble(),
                        kDtheta.getAsDouble(),
                        new TrapezoidProfile.Constraints(3, 13));
                azimuthMovingPidTrap.enableContinuousInput(-Math.PI, Math.PI);
                azimuthMovingPidTrap.setTolerance(0.001, 0.01);
                azimuthMovingPidTrap.reset(
                    new TrapezoidProfile.State(
                        drive.getRotation().getRadians(),
                        drive.getChassisSpeeds().omegaRadiansPerSecond));

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
                for (int a = 0; a < 3; a++) {
                  velXinputs.addLast(MetersPerSecond.of(0.0));
                  velYinputs.addLast(MetersPerSecond.of(0.0));
                }
              },
              drive,
              shooter)
          .andThen(
              Commands.runEnd(
                  () -> {
                    Threads.setCurrentThreadPriority(true, 80);
                    var shooterPos = calcShooterPos(drive.getPose());
                    double xPosFuture =
                        shooterPos.getX()
                            + integrate(appliedVelX, deltaT.in(Seconds), numAdvanceVel);
                    double yPosFuture =
                        shooterPos.getY()
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
                        limitVelocityByAccel(vIn, (velXinputs.peekLast()), (velYinputs.peekLast()));
                    velXinputs.pollFirst();
                    velYinputs.pollFirst();
                    velXinputs.addLast(vLimited.getFirst());
                    velYinputs.addLast(vLimited.getSecond());
                    LinearVelocity velX = velXinputs.get(0);
                    LinearVelocity velY = velYinputs.get(0);

                    double startSolve = HALUtil.getFPGATime();
                    var targetFuture = calcTargetDisplacement(xPosFuture, yPosFuture);
                    TrajectorySolution solution =
                        nodesJNI.newtonRhapsonSolveAirResistance(
                            velX.in(MetersPerSecond),
                            velY.in(MetersPerSecond),
                            targetFuture.getX(),
                            targetFuture.getY(),
                            targetFuture.getZ(),
                            shooterAltitude.in(Radians));
                    double endSolve = HALUtil.getFPGATime();
                    Logger.recordOutput(
                        "moveShootCommand/Calculated/solveTimeMS",
                        (endSolve - startSolve) / 1000.0);

                    AngularVelocity shooterAngularVelocity =
                        RadiansPerSecond.of(solution.shooterVel * kShooter.getAsDouble());
                    Angle theta =
                        Radians.of((solution.azimuth + Math.PI) % (2 * Math.PI) - Math.PI);
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
                    omega = computeOmega(drive, shooter, kShooter);
                    Logger.recordOutput("moveShootCommand/Applied/omega", omega);

                    var speedLinear =
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            velX, velY, RadiansPerSecond.of(omega), drive.getRotation());
                    var speedRotation = new ChassisSpeeds(0, -omega * shooterOffset.getX(), omega);

                    drive.runVelocityDangerous(speedLinear.plus(speedRotation));

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
                        RadiansPerSecond.of(solution.shooterVel * kShooter.getAsDouble());
                    if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
                      if (shooterVel.in(RadiansPerSecond) > 0
                          && shooterVel.in(RadiansPerSecond) < 500)
                        shooter.setShooterVelocityAndAcceleration(shooterVel, lastShooterVel);
                    } else if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
                      if (shooterVel.in(RadiansPerSecond) > 0
                          && shooterVel.in(RadiansPerSecond) < 500)
                        shooter.setShooterVelocityAndAcceleration(
                            shooterVel.times(1.0231944547), lastShooterVel.times(1.0231944547));
                    }
                    Logger.recordOutput(
                        "moveShootCommand/Applied/shooterVel", shooterVel.in(RadiansPerSecond));
                    logRealPlusError(drive, shooter, kShooter);
                    lastShooterVel = shooterVel;
                    Threads.setCurrentThreadPriority(true, 10);
                  },
                  () -> {
                    drive.stop();
                    shooter.setShooterVelocity(RadiansPerSecond.of(0));
                  },
                  drive,
                  shooter));

    } catch (Exception e) {
      DriverStation.reportError("moveShootCommand", e.getStackTrace());
      return Commands
          .none(); // catches exception in command creation during boot, prevents BOOT LOOP
    }
  }

  static AngularVelocity lastShooterVel = RadiansPerSecond.of(0);

  private static int numLagInputs = 10;
  private static int numAdvanceVel =
      Constants.CURRENT_MODE == Constants.Mode.SIM ? 6 : 4; // compensating for pid lag
  private static Time deltaT = Milliseconds.of(20);

  private static int numAdvanceOmega =
      Constants.CURRENT_MODE == Constants.Mode.SIM ? 6 : 8; // compensating for pid lag
  private static int numAdvanceShooter = 4;

  public static void logRealPlusError(Drive drive, Shooter shooter, DoubleSupplier kShooter) {

    /*
     * record real state
     *
     */
    Logger.recordOutput("moveShootCommand/Real/posX", drive.getPose().getMeasureX());
    Logger.recordOutput("moveShootCommand/Real/posY", drive.getPose().getMeasureY());

    ChassisSpeeds realVel =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drive.getChassisSpeeds(),
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());
    Logger.recordOutput("moveShootCommand/Real/vX", MetersPerSecond.of(realVel.vxMetersPerSecond));
    Logger.recordOutput("moveShootCommand/Real/vY", MetersPerSecond.of(realVel.vyMetersPerSecond));
    Logger.recordOutput(
        "moveShootCommand/Real/omega",
        RadiansPerSecond.of(drive.getChassisSpeeds().omegaRadiansPerSecond));
    AngularVelocity shooterVel = shooter.inputs1.shooterVelocityRadPerSec;
    if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
      // shooterVel = shooterVel.times(260 / 16.0 * 200.0 / 204);
    }
    Logger.recordOutput("moveShootCommand/Real/shooterVel", shooterVel.in(RadiansPerSecond));
    Logger.recordOutput("moveShootCommand/Real/theta", drive.getPose().getRotation());

    /*
     *
     * calculate error
     *
     *
     *
     */

    // Distance error =
    //     ProjectileTrajectoryUtils.minDistTrajectory(
    //         MetersPerSecond.of(realVel.vxMetersPerSecond),
    //         MetersPerSecond.of(realVel.vyMetersPerSecond),
    //         hubLocation.minus(
    //             shooterOffset.plus(
    //                 new Translation3d(drive.getPose().getX(), drive.getPose().getY(), 0))),
    //         shooterAltitude,
    //         Radians.of(drive.getPose().getRotation().getRadians()),
    //         MetersPerSecond.of(shooterVel.in(RadiansPerSecond) / kShooter.getAsDouble()));

    // Matrix<N2, N1> displacement =
    //     ProjectileTrajectoryUtils.AirResistanceSolver.f_airResistance_RK4(
    //         new Matrix<>(
    //             Nat.N2(),
    //             Nat.N1(),
    //             new double[] {
    //               shooterVel.in(RadiansPerSecond) / kShooter.getAsDouble(),
    //               drive.getPose().getRotation().getRadians()
    //             }),
    //         realVel.vxMetersPerSecond,
    //         realVel.vyMetersPerSecond,
    //         hubLocation.getZ() - shooterOffset.getZ(),
    //         shooterAltitude.in(Radians));
    var shooterPos = calcShooterPos(drive.getPose());
    Translation3d target = calcTargetDisplacement(shooterPos.getX(), shooterPos.getY());
    Logger.recordOutput(
        "eSim/shooterVel", shooterVel.in(RadiansPerSecond) / kShooter.getAsDouble());
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
        "moveShootCommand/RealCalc/shooterVel", idealNow.shooterVel * kShooter.getAsDouble());

    double error =
        nodesJNI.minDistTrajectory(
            shooterVel.in(RadiansPerSecond) / kShooter.getAsDouble(),
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
            shooterVel.in(RadiansPerSecond) / kShooter.getAsDouble(),
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

  //   Torque maxTorque = NewtonMeters.of(10);
  //   Force maxForce = Newtons.of(10);
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
