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

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.ProjectileSpeedUtils;
import frc.robot.util.ProjectileTrajectoryUtils;
import frc.robot.util.ProjectileTrajectoryUtils.FixedTrajectorySolution;
import org.littletonrobotics.junction.Logger;

public class TestCommands {
  private TestCommands() {}

  private static final Translation3d hubLocation = new Translation3d(0, 0, 2);
  private static final Translation3d shooterOffset =
      new Translation3d(0, -Inches.of(0.1956095).in(Meters), Inches.of(16.081505).in(Meters));
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

  public static Command testAutoAlign(Drive drive, Shooter shooter, double kShooter, double displaceX) {

    return Commands.run(
        () -> {
          double start = HALUtil.getFPGATime();

          // TODO: essentialy euler's method, tracking position, velocity, angle, angular velocity,
          // capping actual acceleration & angular acceleration by computed max values
          Translation2d fieldPos = drive.getPose().getTranslation();
          Translation3d targetDisplacement =
              shooterOffset.plus(new Translation3d(fieldPos.getX(), fieldPos.getY(), 0));
          // lunar converter 152cm bottom - 203cm top
          targetDisplacement = new Translation3d(displaceX, 0, (2.03+1.52)/2);

          FixedTrajectorySolution solution =
              ProjectileTrajectoryUtils.calcFiringSolution(
                  MetersPerSecond.of(0),
                  MetersPerSecond.of(0),
                  targetDisplacement,
                  shooterAltitude);

          double shooterSpeedTransfer =
              ProjectileSpeedUtils.calcSpeedTransferPercentage(
                  ShooterConstants.ShooterSimConstants.SHOOTER_MOI,
                  Pounds.of(0.2),
                  Inches.of(3.0 / 2));
          shooterSpeedTransfer *= kShooter;
          AngularVelocity shooterAngularVelocity =
              ProjectileSpeedUtils.calcNecessaryWheelSpeed(
                  solution.shooterVelocity, shooterSpeedTransfer, Inches.of(3.0 / 2));

          Logger.recordOutput(
              "DriveTest/ShooterRotationsPerSecond", shooterAngularVelocity.in(RotationsPerSecond));
          shooter.setShooterVelocity(shooterAngularVelocity);

          Angle azimuthDiff =
              Radians.of(
                  deadzone(
                      (solution.azimuth.in(Radians) - drive.getPose().getRotation().getRadians()),
                      0.05));

          AngularVelocity omega = azimuthDiff.div(Seconds.of(1.0));
          if (Math.abs(omega.in(RadiansPerSecond)) > drive.getMaxAngularSpeedRadPerSec()) {
            omega =
                RadiansPerSecond.of(
                    Math.copySign(drive.getMaxAngularSpeedRadPerSec(), omega.in(RadiansPerSecond)));
          }
          drive.runVelocity(new ChassisSpeeds(MetersPerSecond.of(0), MetersPerSecond.of(0), omega));
          Logger.recordOutput("DriveTest/AngularSpeed", omega.in(RadiansPerSecond));
          lastOmega = omega;

          Logger.recordOutput("DriveTest/azimuthDiff", azimuthDiff.in(Degrees));
        },
        drive,
        shooter);
  }
}
