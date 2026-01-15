package frc.robot.util;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

public class ProjectileTrajectoryUtils {
  public static class TrajectorySolution {
    public Angle azimuth;
    public LinearVelocity shooterVelocity;

    public TrajectorySolution(Angle azimuth, LinearVelocity shooterVelocity) {
      this.azimuth = azimuth;
      this.shooterVelocity = shooterVelocity;
    }
  }

  private static final LinearAcceleration GRAVITY = MetersPerSecondPerSecond.of(9.81);

  /**
   * Calculates the Robot firing solution
   *
   * @param botVelocityX X-velocity of the Robot, FOC, in meters
   * @param botVelocityY Y-velocity of the Robot, FOC, in meters
   * @param timeOfFlight duration of the delta-t from launch to target
   * @param shooterVelocity exit velocity from the shooter
   * @param targetPos 3d displacement of the target, FOC, in meters
   * @param shooterAltitude Angle of the fixed shooter from horizontal
   * @return Time of flight for the projectile from launch to target
   */
  public static TrajectorySolution calcFiringSolution(
      LinearVelocity botVelocityX,
      LinearVelocity botVelocityY,
      Translation3d targetPos,
      Angle shooterAltitude) {
    Time timeOfFlight = calcTargetTime(botVelocityX, botVelocityY, targetPos, shooterAltitude);
    LinearVelocity shooterVelocity = calcShooterVelocity(timeOfFlight, targetPos, shooterAltitude);
    Angle azimuth =
        calcRobotHeadingAzimuth(
            botVelocityX, botVelocityY, timeOfFlight, shooterVelocity, targetPos, shooterAltitude);
    return new TrajectorySolution(azimuth, shooterVelocity);
  }

  /**
   * Calculates the Robot heading azimuth
   *
   * @param botVelocityX X-velocity of the Robot, FOC, in meters
   * @param botVelocityY Y-velocity of the Robot, FOC, in meters
   * @param timeOfFlight duration of the delta-t from launch to target
   * @param shooterVelocity exit velocity from the shooter
   * @param targetPos 3d displacement of the target, FOC, in meters
   * @param shooterAltitude Angle of the fixed shooter from horizontal
   * @return Time of flight for the projectile from launch to target
   */
  public static Angle calcRobotHeadingAzimuth(
      LinearVelocity botVelocityX,
      LinearVelocity botVelocityY,
      Time timeOfFlight,
      LinearVelocity shooterVelocity,
      Translation3d targetPos,
      Angle shooterAltitude) {
    double g = GRAVITY.in(MetersPerSecondPerSecond);
    double sinTheta =
        targetPos.getY() - timeOfFlight.in(Seconds) * botVelocityY.in(MetersPerSecond);
    double cosTheta =
        targetPos.getX() - timeOfFlight.in(Seconds) * botVelocityX.in(MetersPerSecond);
    Logger.recordOutput("Trajectory/AzimuthSolution/sinTheta", sinTheta);
    Logger.recordOutput("Trajectory/AzimuthSolution/cosTheta", cosTheta);
    return Radians.of(Math.atan2(sinTheta, cosTheta));
  }

  public static AngularVelocity calcRobotAngularVelocity(
      LinearVelocity botVelocityX,
      LinearVelocity botVelocityY,
      Time timeOfFlight,
      double dfdt,
      Translation3d targetPos,
      Angle shooterAltitude) {
    double g = GRAVITY.in(MetersPerSecondPerSecond);
    double term1 = g / (2 * Math.sin(shooterAltitude.in(Radians)));
    double term2 =
        -targetPos.getZ()
            / (Math.pow(timeOfFlight.in(Seconds), 2.0) * Math.sin(shooterAltitude.in(Radians)));
    return RadiansPerSecond.of(dfdt * (term1 + term2));
  }

  /**
   * Calculates the Shooter exit velocity for the projectile
   *
   * @param timeOfFlight duration of the delta-t from launch to target
   * @param targetPos 3d displacement of the target, FOC, in meters
   * @param shooterAltitude Angle of the fixed shooter from horizontal
   * @return Time of flight for the projectile from launch to target
   */
  public static LinearVelocity calcShooterVelocity(
      Time timeOfFlight, Translation3d targetPos, Angle shooterAltitude) {
    double g = GRAVITY.in(MetersPerSecondPerSecond);
    double numerator = targetPos.getZ() + 0.5 * g * Math.pow(timeOfFlight.in(Seconds), 2.0);
    // Distance numerator =
    // Meters.of(targetPos.getZ()).plus(GRAVITY.times(timeOfFlight.times(timeOfFlight).times(0.5)));
    double denominator = timeOfFlight.in(Seconds) * Math.sin(shooterAltitude.in(Radians));
    Logger.recordOutput("Trajectory/VelocitySolution/numerator", numerator);
    Logger.recordOutput("Trajectory/VelocitySolution/denominator", denominator);
    return MetersPerSecond.of(numerator / denominator);
  }

  public static LinearAcceleration calcShooterAcceleration(
      Time timeOfFlight, double dfdt, Translation3d targetPos, Angle shooterAltitude) {
    double g = GRAVITY.in(MetersPerSecondPerSecond);
    double term1 = g / (2 * Math.sin(shooterAltitude.in(Radians)));
    double term2 =
        -targetPos.getZ()
            / (Math.pow(timeOfFlight.in(Seconds), 2.0) * Math.sin(shooterAltitude.in(Radians)));
    return MetersPerSecondPerSecond.of(dfdt * (term1 + term2));
  }

  /**
   * Calculates the Time of flight for the projectile from launch to target
   *
   * @param botVelocityX X-velocity of the Robot, FOC, in meters
   * @param botVelocityY Y-velocity of the Robot, FOC, in meters
   * @param targetPos 3d displacement of the target, FOC, in meters
   * @param shooterAltitude Angle of the fixed shooter from horizontal
   * @return Time of flight for the projectile from launch to target
   */
  public static Time calcTargetTime(
      LinearVelocity botVelocityX,
      LinearVelocity botVelocityY,
      Translation3d targetPos,
      Angle shooterAltitude) {
    double g = GRAVITY.in(MetersPerSecondPerSecond);
    double cos2a = Math.pow(Math.cos(shooterAltitude.in(Radians)), 2);
    double sin2a = Math.pow(Math.sin(shooterAltitude.in(Radians)), 2);
    double v_x = botVelocityX.in(MetersPerSecond);
    double v_y = botVelocityY.in(MetersPerSecond);
    double a = 0.25 * Math.pow(g, 2) * cos2a;
    double b = g * targetPos.getZ() * cos2a - v_x * v_x * sin2a - v_y * v_y * sin2a;
    double c = 2.0 * (targetPos.getX() * v_x + targetPos.getY() * v_y) * sin2a;
    double d =
        Math.pow(targetPos.getZ(), 2) * cos2a
            - Math.pow(targetPos.getX(), 2) * sin2a
            - Math.pow(targetPos.getY(), 2) * sin2a;
    Logger.recordOutput("Trajectory/QuarticSolution/a", a);
    Logger.recordOutput("Trajectory/QuarticSolution/b", b);
    Logger.recordOutput("Trajectory/QuarticSolution/c", c);
    Logger.recordOutput("Trajectory/QuarticSolution/d", d);
    if (c == 0) {
      return Time.ofBaseUnits(Math.sqrt(solveQuadraticRealHigh(a, b, d)), Seconds);
    } else {
      return Time.ofBaseUnits(solveDepressedQuarticRealHigh(a, b, c, d), Seconds);
    }
  }

  public static double solveQuartic(double a, double b, double c, double d, double e) {
    return 0;
  }

  public static double solveQuadraticRealHigh(double a, double b, double c) {
    if (a > 0) return (-b + Math.sqrt(b * b - 4 * a * c)) / (2 * a);
    else return (-b - Math.sqrt(b * b - 4 * a * c)) / (2 * a);
  }

  public static double calcDfdt(
      LinearVelocity botVelocityX,
      LinearVelocity botVelocityY,
      Translation3d targetPos,
      Angle shooterAltitude) {
    double g = GRAVITY.in(MetersPerSecondPerSecond);
    double cos2a = Math.pow(Math.cos(shooterAltitude.in(Radians)), 2);
    double sin2a = Math.pow(Math.sin(shooterAltitude.in(Radians)), 2);
    double x = targetPos.getX();
    double y = targetPos.getY();
    double v_x = botVelocityX.in(MetersPerSecond);
    double v_y = botVelocityY.in(MetersPerSecond);
    double a = 0.25 * Math.pow(g, 2) * cos2a;
    double b = g * targetPos.getZ() * cos2a - v_x * v_x * sin2a - v_y * v_y * sin2a;
    double c = 2.0 * (x * v_x + y * v_y) * sin2a;
    double d = Math.pow(targetPos.getZ(), 2) * cos2a - x * x * sin2a - y * y * sin2a;

    double accel_x = 0;
    double accel_y = 0;
    double bPrime = -2 * sin2a * v_x - 2 * sin2a * v_y;
    double cPrime = 2 * sin2a * (v_x * v_x + accel_x * x + v_y * v_y + accel_y * y);
    double dPrime = -2 * sin2a * x * v_x - 2 * sin2a * y * v_y;
    return dfdtDepressedQuarticRealHigh(a, b, c, d, bPrime, cPrime, dPrime);
  }

  public static double dfdtDepressedQuarticRealHigh(
      double a, double b, double c, double d, double bPrime, double cPrime, double dPrime) {
    double h = 1e-4;
    double root1 = solveDepressedQuarticRealHigh(a, b, c, d);
    double root2 = solveDepressedQuarticRealHigh(a, b + bPrime * h, c + cPrime * h, d + dPrime * h);
    double numericalDerivative = (root2 - root1) / h;
    Logger.recordOutput("Trajectory/Numerical Derivative e-4: ", numericalDerivative);
    h = 1e-6;
    root2 = solveDepressedQuarticRealHigh(a, b + bPrime * h, c + cPrime * h, d + dPrime * h);
    numericalDerivative = (root2 - root1) / h;
    Logger.recordOutput("Trajectory/Numerical Derivative e-6: ", numericalDerivative);
    double dfdt = numericalDerivative;
    return dfdt;
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
