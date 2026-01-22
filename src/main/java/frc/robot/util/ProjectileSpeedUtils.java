package frc.robot.util;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import java.util.Optional;

// Source: https://www.reca.lc/flywheel
/**
 * Utility class for calculating projectile speeds and necessary wheel speeds
 *
 * @author Kavin Muralikrishnan
 * @see <a href="https://www.reca.lc/flywheel">Flywheel projectile calculation source</a>
 */
public final class ProjectileSpeedUtils {
  private static final LinearAcceleration GRAVITY = MetersPerSecondPerSecond.of(9.81);
  private static final double FEET_PER_SECOND_CONVERSION = 114.6;
  private static final double[] SPEED_TRANSFER_PERCENTAGE_CONSTANTS = {20, 28, 40};
  /**
   * Calculates the percentage of speed transferred from the shooter wheel to the projectile
   *
   * @param totalMOI Total moment of inertia of flywheel and shooter wheel, equal to the sum of both
   *     individual MOIs
   * @param projectileWeight Weight of the projectile being shot
   * @param shooterWheelRadius Radius of the shooter wheel
   * @return Percentage of speed transferred from shooter wheel to projectile (0-1)
   * @see <a href="https://www.reca.lc/flywheel">Flywheel projectile calculation source</a>
   */
  public static double calcSpeedTransferPercentage(
      MomentOfInertia totalMOI, Mass projectileWeight, Distance shooterWheelRadius) {
    return SPEED_TRANSFER_PERCENTAGE_CONSTANTS[0]
        * totalMOI.in(KilogramSquareMeters)
        / (SPEED_TRANSFER_PERCENTAGE_CONSTANTS[1]
                * projectileWeight.in(Kilograms)
                * (shooterWheelRadius.in(Meters) * shooterWheelRadius.in(Meters))
            + SPEED_TRANSFER_PERCENTAGE_CONSTANTS[2] * totalMOI.in(KilogramSquareMeters));
  }

  /**
   * Calculates the percentage of speed transferred from the shooter wheel to the projectile
   *
   * @param shooterWheelMOI Moment of Inertia of the shooter wheel
   * @param flywheelMOI Moment of Inertia of the flywheel
   * @param projectileWeight Weight of the projectile being shot
   * @param shooterWheelRadius Radius of the shooter wheel
   * @return Percentage of speed transferred from shooter wheel to projectile (0-1)
   * @see <a href="https://www.reca.lc/flywheel">Flywheel projectile calculation source</a>
   */
  public static double calcSpeedTransferPercentage(
      MomentOfInertia shooterWheelMOI,
      MomentOfInertia flywheelMOI,
      Mass projectileWeight,
      Distance shooterWheelRadius) {
    return calcSpeedTransferPercentage(
        shooterWheelMOI.plus(flywheelMOI), projectileWeight, shooterWheelRadius);
  }

  /**
   * Calculates the necessary wheel speed to achieve a desired projectile speed
   *
   * @param desiredProjectileSpeed Desired speed of the projectile
   * @param speedTransferPercentage Percentage of speed transferred from shooter wheel to projectile
   *     (0-1).
   * @param shooterWheelRadius Radius of the shooter wheel
   * @return Necessary wheel speed to achieve desired projectile speed
   * @see <a href="https://www.reca.lc/flywheel">Flywheel projectile calculation source</a>
   * @see #calcSpeedTransferPercentage(MomentOfInertia, Mass, Distance)
   * @see #calcSpeedTransferPercentage(MomentOfInertia, MomentOfInertia, Mass, Distance)
   */
  public static AngularVelocity calcNecessaryWheelSpeed(
      LinearVelocity desiredProjectileSpeed,
      double speedTransferPercentage,
      Distance shooterWheelRadius) {
    return RPM.of(
        desiredProjectileSpeed.in(FeetPerSecond)
            * FEET_PER_SECOND_CONVERSION
            / (speedTransferPercentage * shooterWheelRadius.in(Inches)));
  }

  /**
   * Calculates the necessary wheel speed to achieve a desired projectile speed
   *
   * @param desiredProjectileSpeed Desired speed of the projectile
   * @param shooterWheelMOI Moment of Inertia of the shooter wheel
   * @param flywheelMOI Moment of Inertia of the flywheel
   * @param projectileWeight Weight of the projectile being shot
   * @param shooterWheelRadius Radius of the shooter wheel
   * @return Necessary wheel speed to achieve desired projectile speed
   * @see <a href="https://www.reca.lc/flywheel">Flywheel projectile calculation source</a>
   */
  public static AngularVelocity calcNecessaryWheelSpeed(
      LinearVelocity desiredProjectileSpeed,
      MomentOfInertia shooterWheelMOI,
      MomentOfInertia flywheelMOI,
      Mass projectileWeight,
      Distance shooterWheelRadius) {
    double speedTransferPercentage =
        calcSpeedTransferPercentage(
            shooterWheelMOI, flywheelMOI, projectileWeight, shooterWheelRadius);
    return calcNecessaryWheelSpeed(
        desiredProjectileSpeed, speedTransferPercentage, shooterWheelRadius);
  }

  /**
   * Calculates the necessary wheel speed to achieve a desired projectile speed
   *
   * @param desiredProjectileSpeed Desired speed of the projectile
   * @param totalMOI Total moment of inertia of flywheel and shooter wheel, equal to the sum of both
   *     individual MOIs
   * @param projectileWeight Weight of the projectile being shot
   * @param shooterWheelRadius Radius of the shooter wheel
   * @return Necessary wheel speed to achieve desired projectile speed
   * @see <a href="https://www.reca.lc/flywheel">Flywheel projectile calculation source</a>
   */
  public static AngularVelocity calcNecessaryWheelSpeed(
      LinearVelocity desiredProjectileSpeed,
      MomentOfInertia totalMOI,
      Mass projectileWeight,
      Distance shooterWheelRadius) {
    double speedTransferPercentage =
        calcSpeedTransferPercentage(totalMOI, projectileWeight, shooterWheelRadius);
    return calcNecessaryWheelSpeed(
        desiredProjectileSpeed, speedTransferPercentage, shooterWheelRadius);
  }

  /**
   * Calculates the projectile speed achieved from a given wheel speed
   *
   * @param wheelSpeed Shooter wheel speed
   * @param speedTransferPercentage Percentage of speed transferred from shooter wheel to projectile
   *     (0-1)
   * @param shooterWheelRadius Radius of the shooter wheel
   * @return Achieved projectile speed
   * @see <a href="https://www.reca.lc/flywheel">Flywheel projectile calculation source</a>
   * @see #calcSpeedTransferPercentage(MomentOfInertia, Mass, Distance)
   * @see #calcSpeedTransferPercentage(MomentOfInertia, MomentOfInertia, Mass, Distance)
   */
  public static LinearVelocity calcProjectileSpeed(
      AngularVelocity wheelSpeed, double speedTransferPercentage, Distance shooterWheelRadius) {
    return MetersPerSecond.of(
        wheelSpeed.in(RPM)
            * speedTransferPercentage
            * shooterWheelRadius.in(Inches)
            / FEET_PER_SECOND_CONVERSION);
  }

  /**
   * Calculates the projectile speed achieved from a given wheel speed
   *
   * @param wheelSpeed Shooter wheel speed
   * @param shooterWheelMOI Moment of Inertia of the shooter wheel
   * @param flywheelMOI Moment of Inertia of the flywheel
   * @param projectileWeight Weight of the projectile being shot
   * @param shooterWheelRadius Radius of the shooter wheel
   * @return Achieved projectile speed
   * @see <a href="https://www.reca.lc/flywheel">Flywheel projectile calculation source</a>
   */
  public static LinearVelocity calcProjectileSpeed(
      AngularVelocity wheelSpeed,
      MomentOfInertia shooterWheelMOI,
      MomentOfInertia flywheelMOI,
      Mass projectileWeight,
      Distance shooterWheelRadius) {
    double speedTransferPercentage =
        calcSpeedTransferPercentage(
            shooterWheelMOI, flywheelMOI, projectileWeight, shooterWheelRadius);
    return calcProjectileSpeed(wheelSpeed, speedTransferPercentage, shooterWheelRadius);
  }

  /**
   * Calculates the projectile speed achieved from a given wheel speed
   *
   * @param wheelSpeed Shooter wheel speed
   * @param totalMOI Total moment of inertia of flywheel and shooter wheel, equal to the sum of both
   *     individual MOIs
   * @param projectileWeight Weight of the projectile being shot
   * @param shooterWheelRadius Radius of the shooter wheel
   * @return Achieved projectile speed
   * @see <a href="https://www.reca.lc/flywheel">Flywheel projectile calculation source</a>
   */
  public static LinearVelocity calcProjectileSpeed(
      AngularVelocity wheelSpeed,
      MomentOfInertia totalMOI,
      Mass projectileWeight,
      Distance shooterWheelRadius) {
    double speedTransferPercentage =
        calcSpeedTransferPercentage(totalMOI, projectileWeight, shooterWheelRadius);
    return calcProjectileSpeed(wheelSpeed, speedTransferPercentage, shooterWheelRadius);
  }

  /**
   * Calculates the necessary projectile speed to reach a target from a given pose
   *
   * @param initialProjectilePose Pose of the projectile at launch
   * @param targetPose Pose of the target
   * @param launchAngle Angle at which the projectile is launched
   * @param maxSpeed Maximum speed of the projectile
   * @return Necessary projectile speed to reach the target, or -1 if it's not possible within max
   *     speed or constraints
   * @see <a
   *     href="https://physics.stackexchange.com/questions/27992/solving-for-initial-velocity-required-to-launch-a-projectile-to-a-given-destinat/27993#27993">Projectile
   *     motion source</a>
   */
  public static Optional<Double> calcNecessaryProjectileSpeed(
      Pose3d initialProjectilePose,
      Pose3d targetPose,
      Angle launchAngle,
      LinearVelocity maxSpeed,
      Angle directionalTolerance) {
    // Checks that the robot is facing towards the target within the tolerance
    if (targetPose
            .getTranslation()
            .toTranslation2d()
            .minus(initialProjectilePose.getTranslation().toTranslation2d())
            .getAngle()
            .minus(initialProjectilePose.getRotation().toRotation2d())
            .getRotations()
        > directionalTolerance.in(Rotations)) return Optional.empty();

    // Distance between robot and target in the X-Y plane
    double dX =
        Math.sqrt(
            (targetPose.getX() - initialProjectilePose.getX())
                    * (targetPose.getX() - initialProjectilePose.getX())
                + (targetPose.getY() - initialProjectilePose.getY())
                    * (targetPose.getY() - initialProjectilePose.getY()));
    // Height difference
    double dZ = targetPose.getZ() - initialProjectilePose.getZ();

    double calcVelocity =
        (1 / Math.cos(launchAngle.in(Radians)))
            * Math.sqrt(
                ((GRAVITY.in(MetersPerSecondPerSecond) / 2) * dX * dX)
                    / (dX * Math.tan(launchAngle.in(Radians)) - dZ));

    // If the calculated velocity is greater than the max speed, return -1 to
    // indicate it's not possible
    return (calcVelocity) > maxSpeed.in(MetersPerSecond)
        ? Optional.empty()
        : Optional.of(calcVelocity);
  }
}
