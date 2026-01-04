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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public class ModuleIO {
  @AutoLog
  static class ModuleIOInputs {
    public boolean driveConnected = false;
    public Angle drivePositionRad = Radians.of(Double.NaN);
    public AngularVelocity driveVelocityRadPerSec = RadiansPerSecond.of(Double.NaN);
    public Voltage driveAppliedVolts = Volts.of(Double.NaN);
    public Current driveCurrentAmps = Amps.of(Double.NaN);

    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;
    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public AngularVelocity turnVelocityRadPerSec = RadiansPerSecond.of(Double.NaN);
    public Voltage turnAppliedVolts = Volts.of(Double.NaN);
    public Current turnCurrentAmps = Amps.of(Double.NaN);

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  void setDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  void setTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  void setDriveVelocity(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  void setTurnPosition(Rotation2d rotation) {}

  void reconfigureDriveMotor() {}

  void reconfigureTurnMotor() {}
}
