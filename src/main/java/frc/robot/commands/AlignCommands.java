package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.ProjectileSpeedUtils;
import frc.robot.util.ProjectileTrajectoryUtils;
import frc.robot.util.ProjectileTrajectoryUtils.FixedTrajectorySolution;
import org.littletonrobotics.junction.Logger;

public class AlignCommands {
  private AlignCommands() {}

  private static Translation3d hubLocation = new Translation3d(0, 0, (2.03 + 1.52) / 2); // lunar converter 152cm bottom - 203cm top
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

  public static Command resetDisplacement(Drive drive, double displacementX) {
    try{
    return Commands.runOnce(
        () -> {
          Translation2d fieldPos = drive.getPose().getTranslation();
          hubLocation =
              new Translation3d(
                  fieldPos.getX() + displacementX,
                  fieldPos.getY(),
                  (2.03 + 1.52) / 2); // lunar converter 152cm bottom - 203cm top
        },
        drive);
    } catch (Exception e) {
        DriverStation.reportError(
            "Failed to create resetDisplacement command",
            e.getStackTrace()
        );
        return Commands.none(); // catches exception in command creation during boot, prevents BOOT LOOP
    }
  }

  public static Command testAlignStationary(
      Drive drive, Shooter shooter, double kShooter, double displacementX) {
try{
    return 
            Commands.run(
                () -> {
                  double start = HALUtil.getFPGATime();
                  Translation2d fieldPos = drive.getPose().getTranslation();
                  Translation3d targetDisplacement =
                      hubLocation.minus(
                          shooterOffset.plus(
                              new Translation3d(fieldPos.getX(), fieldPos.getY(), 0)));
                  FixedTrajectorySolution solution =
                      ProjectileTrajectoryUtils.calcFiringSolution(
                          MetersPerSecond.of(0),
                          MetersPerSecond.of(0),
                          targetDisplacement,
                          shooterAltitude);
                  AngularVelocity shooterAngularVelocity =
                      RadiansPerSecond.of(
                          solution.shooterVelocity.in(MetersPerSecond) * kShooter);

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
                shooter);
  
    } catch (Exception e) {
        DriverStation.reportError(
            "Failed to create testAutoAlign command",
            e.getStackTrace()
        );
        return Commands.none(); // catches exception in command creation during boot, prevents BOOT LOOP
    }
  }
}
