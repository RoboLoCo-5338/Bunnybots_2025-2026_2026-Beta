package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ProjectileTrajectoryUtils;
import frc.robot.util.ProjectileTrajectoryUtils.FixedTrajectorySolution;
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
  private static int numLagFrames = 3;

  private static double deadzone(double input, double zone) {
    if (Math.abs(input) < zone) {
      return 0.0;
    } else {
      return input;
    }
  }

  public static Command resetDisplacement(Drive drive, DoubleSupplier displacementX) {
    try {
      return Commands.runOnce(
          () -> {
            Translation2d fieldPos = drive.getPose().getTranslation();
            hubLocation =
                new Translation3d(
                    fieldPos.getX() + displacementX.getAsDouble(),
                    fieldPos.getY(),
                    (2.03 + 1.52) / 2); // lunar converter 152cm bottom - 203cm top
          },
          drive);
    } catch (Exception e) {
      DriverStation.reportError("Failed to create resetDisplacement command", e.getStackTrace());
      return Commands
          .none(); // catches exception in command creation during boot, prevents BOOT LOOP
    }
  }

  static final double azimuthKp = 7.0;
  static final double azimuthKd = 1.0;
  static PIDController azimuthStationaryPid = new PIDController(azimuthKp, 0, azimuthKd);

  public static Command testAlignStationary(Drive drive, Shooter shooter, DoubleSupplier kShooter) {
    try {
      return Commands.runOnce(
              () -> {
                azimuthStationaryPid = new PIDController(azimuthKp, 0, azimuthKd);
                azimuthStationaryPid.enableContinuousInput(-Math.PI, Math.PI);
              })
          .andThen(
              Commands.run(
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

                    AngularVelocity omega = azimuthDiff.div(Seconds.of(0.5));
                    if (Math.abs(omega.in(RadiansPerSecond))
                        > drive.getMaxAngularSpeedRadPerSec()) {
                      omega =
                          RadiansPerSecond.of(
                              Math.copySign(
                                  drive.getMaxAngularSpeedRadPerSec(), omega.in(RadiansPerSecond)));
                    }
                    omega =
                        RadiansPerSecond.of(
                            azimuthStationaryPid.calculate(
                                drive.getRotation().getRadians(), solution.azimuth.in(Radians)));
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
                  drive,
                  shooter));

    } catch (Exception e) {
      DriverStation.reportError("Failed to create AlignStationary command", e.getStackTrace());
      return Commands
          .none(); // catches exception in command creation during boot, prevents BOOT LOOP
    }
  }

  // static PIDController azimuthMovingPid = new PIDController(azimuthKp, 0, azimuthKd);
  public static double startAlignMovingTime = 0.0;
  static Angle setAzimuth = Degrees.of(0);

  static Distance setX = Meters.of(0);
  static Distance setY = Meters.of(0);

  public static Command alignMoving(
      Drive drive,
      Shooter shooter,
      DoubleSupplier kShooter,
      DoubleSupplier displacementX,
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
                    LinearVelocity vX, vY;
                    double vMag =
                        Math.sqrt(
                            inputX.getAsDouble() * inputX.getAsDouble()
                                + inputY.getAsDouble() * inputY.getAsDouble());
                    if (vMag > 1) {
                      vX = maxSpeed.times(inputX.getAsDouble() / vMag);
                      vY = maxSpeed.times(inputY.getAsDouble() / vMag);
                    } else {
                      vX = maxSpeed.times(inputX.getAsDouble());
                      vY = maxSpeed.times(inputY.getAsDouble());
                    }
                    Logger.recordOutput("AlignMoving/vX", vX);
                    Logger.recordOutput("AlignMoving/vY", vY);

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
                        ProjectileTrajectoryUtils.calcMovingFiringSolution(
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
                                : drive.getRotation()).plus(new Rotation2d(omega.times(Milliseconds.of(10)))));

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
                    Logger.recordOutput(
                        "AlignMoving/azimuth",
                        solution.azimuth);
                  },
                  () -> {
                    drive.stop();
                  }));
    } catch (Exception e) {
      DriverStation.reportError("testAlignMoving", e.getStackTrace());
      return Commands
          .none(); // catches exception in command creation during boot, prevents BOOT LOOP
    }
  }
}
