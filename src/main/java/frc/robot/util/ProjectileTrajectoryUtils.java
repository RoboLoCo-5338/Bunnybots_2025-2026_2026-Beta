package frc.robot.util;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

public class ProjectileTrajectoryUtils {
  private static final LinearAcceleration GRAVITY = MetersPerSecondPerSecond.of(9.81);

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
  //   public static double calcSpeedTransferPercentage(
  //       MomentOfInertia totalMOI, Mass projectileWeight, Distance shooterWheelRadius) {
  //     return SPEED_TRANSFER_PERCENTAGE_CONSTANTS[0]
  //         * totalMOI.in(KilogramSquareMeters)
  //         / (SPEED_TRANSFER_PERCENTAGE_CONSTANTS[1]
  //                 * projectileWeight.in(Kilograms)
  //                 * (shooterWheelRadius.in(Meters) * shooterWheelRadius.in(Meters))
  //             + SPEED_TRANSFER_PERCENTAGE_CONSTANTS[2] * totalMOI.in(KilogramSquareMeters));
  //   }
  public static Time calcTargetTime(
      LinearVelocity botVelocityX,
      LinearVelocity botVelocityY,
      Translation3d targetPose,
      Angle shooterAltitude) {
    double g = GRAVITY.in(MetersPerSecondPerSecond);
    double cos2a = Math.pow(Math.cos(shooterAltitude.in(Radians)), 2);
    double sin2a = Math.pow(Math.sin(shooterAltitude.in(Radians)), 2);
    double v_x = botVelocityX.in(MetersPerSecond);
    double v_y = botVelocityY.in(MetersPerSecond);
    double a = 0.25 * Math.pow(g, 2) * cos2a;
    double b =
        GRAVITY.in(MetersPerSecondPerSecond) * targetPose.getZ() * cos2a
            - v_x * v_x * sin2a
            - v_y * v_y * sin2a;
    double c = 2.0 * (targetPose.getX() * v_x + targetPose.getY() * v_y) * sin2a;
    double d =
        Math.pow(targetPose.getZ(), 2) * cos2a
            - Math.pow(targetPose.getX(), 2) * sin2a
            - Math.pow(targetPose.getY(), 2) * sin2a;
    Logger.recordOutput("QuarticSolution/a", a);
    Logger.recordOutput("QuarticSolution/b", b);
    Logger.recordOutput("QuarticSolution/c", c);
    Logger.recordOutput("QuarticSolution/d", d);
    return Time.ofBaseUnits(solveDepressedQuarticRealHigh(a, b, c, d), Seconds);
  }

  public static double solveQuartic(double a, double b, double c, double d, double e) {
    return 0;
  }

  public static double solveQuadraticRealHigh(double a, double b, double c) {
    if (a > 0) return (-b + Math.sqrt(b * b - 4 * a * c)) / (2 * a);
    else return (-b - Math.sqrt(b * b - 4 * a * c)) / (2 * a);
  }

  public static double solveDepressedQuarticRealHigh(double a, double b, double c, double d) {
    return 0.5
            * Math.sqrt(
                -Math.cbrt(
                            Math.sqrt(
                                    Math.pow(-72 * a * b * d + 27 * a * c * c + 2 * b * b * b, 2)
                                        - 4 * Math.pow(12 * a * d + b * b, 3))
                                - 72 * a * b * d
                                + 27 * a * c * c
                                + 2 * b * b * b)
                        / (3 * Math.cbrt(2) * a)
                    - (Math.cbrt(2) * (12 * a * d + b * b))
                        / (3
                            * a
                            * Math.cbrt(
                                Math.sqrt(
                                        Math.pow(
                                                -72 * a * b * d + 27 * a * c * c + 2 * b * b * b, 2)
                                            - 4 * Math.pow(12 * a * d + b * b, 3))
                                    - 72 * a * b * d
                                    + 27 * a * c * c
                                    + 2 * b * b * b))
                    + (2 * c)
                        / (a
                            * Math.sqrt(
                                Math.cbrt(
                                            Math.sqrt(
                                                    Math.pow(
                                                            -72 * a * b * d
                                                                + 27 * a * c * c
                                                                + 2 * b * b * b,
                                                            2)
                                                        - 4 * Math.pow(12 * a * d + b * b, 3))
                                                - 72 * a * b * d
                                                + 27 * a * c * c
                                                + 2 * b * b * b)
                                        / (3 * Math.cbrt(2) * a)
                                    + (Math.cbrt(2) * (12 * a * d + b * b))
                                        / (3
                                            * a
                                            * Math.cbrt(
                                                Math.sqrt(
                                                        Math.pow(
                                                                -72 * a * b * d
                                                                    + 27 * a * c * c
                                                                    + 2 * b * b * b,
                                                                2)
                                                            - 4 * Math.pow(12 * a * d + b * b, 3))
                                                    - 72 * a * b * d
                                                    + 27 * a * c * c
                                                    + 2 * b * b * b))
                                    - (2 * b) / (3.0 * a)))
                    - (4 * b) / (3.0 * a))
        - 0.5
            * Math.sqrt(
                Math.cbrt(
                            Math.sqrt(
                                    Math.pow(-72 * a * b * d + 27 * a * c * c + 2 * b * b * b, 2)
                                        - 4 * Math.pow(12 * a * d + b * b, 3))
                                - 72 * a * b * d
                                + 27 * a * c * c
                                + 2 * b * b * b)
                        / (3 * Math.cbrt(2) * a)
                    + (Math.cbrt(2) * (12 * a * d + b * b))
                        / (3
                            * a
                            * Math.cbrt(
                                Math.sqrt(
                                        Math.pow(
                                                -72 * a * b * d + 27 * a * c * c + 2 * b * b * b, 2)
                                            - 4 * Math.pow(12 * a * d + b * b, 3))
                                    - 72 * a * b * d
                                    + 27 * a * c * c
                                    + 2 * b * b * b))
                    - (2 * b) / (3.0 * a));
  }
}
