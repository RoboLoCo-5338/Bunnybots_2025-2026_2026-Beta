package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class ProjectileTrajectoryUtils {

  private static final int simSteps = 100;

  public static Distance minDistTrajectory(
      LinearVelocity botVelocityX,
      LinearVelocity botVelocityY,
      Translation3d targetPos,
      Angle shooterAltitude,
      Angle azimuth,
      LinearVelocity shooterVelocity) {
    LinearVelocity vX =
        botVelocityX.plus(
            shooterVelocity.times(
                Math.cos(shooterAltitude.in(Radians)) * Math.cos(azimuth.in(Radians))));
    LinearVelocity vY =
        botVelocityY.plus(
            shooterVelocity.times(
                Math.cos(shooterAltitude.in(Radians)) * Math.sin(azimuth.in(Radians))));
    LinearVelocity botVelocity =
        MetersPerSecond.of(
            Math.sqrt(
                Math.pow(botVelocityX.in(MetersPerSecond), 2)
                    + Math.pow(botVelocityY.in(MetersPerSecond), 2)));
    LinearVelocity vR =
        botVelocity.plus(shooterVelocity.times(Math.cos(shooterAltitude.in(Radians))));
    LinearVelocity vZ = shooterVelocity.times(Math.sin(shooterAltitude.in(Radians)));

    double minDist = Double.MAX_VALUE;
    for (int i = 0; i < simSteps; i++) {
      double t = (i + 1.0) * (1.0 / simSteps) * 2.0 * targetPos.getX() / vX.in(MetersPerSecond);
      double x = vX.in(MetersPerSecond) * t;
      double y = vY.in(MetersPerSecond) * t;
      double z = vZ.in(MetersPerSecond) * t - 0.5 * GRAVITY.in(MetersPerSecondPerSecond) * t * t;
      double dist =
          Math.sqrt(
              Math.pow(x - targetPos.getX(), 2)
                  + Math.pow(y - targetPos.getY(), 2)
                  + Math.pow(z - targetPos.getZ(), 2));
      if (dist < minDist) {
        minDist = dist;
      }
    }

    return Meters.of(minDist);
  }

  public static class MovingTrajectorySolution {
    public Angle azimuth;
    public AngularVelocity omega;
    public LinearVelocity shooterVelocity;
    public LinearAcceleration shooterAcceleration;

    public MovingTrajectorySolution(
        Angle azimuth,
        AngularVelocity omega,
        LinearVelocity shooterVelocity,
        LinearAcceleration shooterAcceleration) {
      this.azimuth = azimuth;
      this.shooterVelocity = shooterVelocity;
      this.omega = omega;
      this.shooterAcceleration = shooterAcceleration;
    }
  }
  /**
   * Calculates the Robot moving firing solution
   *
   * @param botVelocityX X-velocity of the Robot, FOC, in meters
   * @param botVelocityY Y-velocity of the Robot, FOC, in meters
   * @param targetPos 3d displacement of the target, FOC, in meters
   * @param shooterAltitude Angle of the fixed shooter from horizontal
   * @return Time of flight for the projectile from launch to target
   */
  public static MovingTrajectorySolution calcMovingFiringSolution(
      LinearVelocity botVelocityX,
      LinearVelocity botVelocityY,
      Translation3d targetPos,
      Angle shooterAltitude) {

    if (Math.sqrt(
            Math.pow(botVelocityX.in(MetersPerSecond), 2)
                + Math.pow(botVelocityY.in(MetersPerSecond), 2))
        < 1e-3) {
      FixedTrajectorySolution fixedSolution =
          calcFiringSolution(botVelocityX, botVelocityY, targetPos, shooterAltitude);
      return new MovingTrajectorySolution(
          fixedSolution.azimuth,
          RadiansPerSecond.of(0),
          fixedSolution.shooterVelocity,
          MetersPerSecondPerSecond.of(0));
    }

    Time timeOfFlight = calcTargetTime(botVelocityX, botVelocityY, targetPos, shooterAltitude);
    Logger.recordOutput("MovingTrajectory/timeOfFlight", timeOfFlight);
    double dfdt = calcDfdt(botVelocityX, botVelocityY, targetPos, shooterAltitude);
    Logger.recordOutput("MovingTrajectory/dfdt", dfdt);
    LinearVelocity shooterVelocity = calcShooterVelocity(timeOfFlight, targetPos, shooterAltitude);
    Logger.recordOutput("MovingTrajectory/shooterVelocity", shooterVelocity);
    LinearAcceleration shooterAcceleration =
        calcShooterAcceleration(timeOfFlight, dfdt, targetPos, shooterAltitude);
    Logger.recordOutput("MovingTrajectory/shooterAcceleration", shooterAcceleration);
    Angle azimuth =
        calcRobotHeadingAzimuth(
            botVelocityX, botVelocityY, timeOfFlight, shooterVelocity, targetPos, shooterAltitude);
    Logger.recordOutput("MovingTrajectory/azimuth", azimuth);
    AngularVelocity omega =
        calcRobotAngularVelocity(
            botVelocityX, botVelocityY, timeOfFlight, dfdt, targetPos, shooterAltitude);
    Logger.recordOutput("MovingTrajectory/omega", omega);
    return new MovingTrajectorySolution(azimuth, omega, shooterVelocity, shooterAcceleration);
  }

  public static class FixedTrajectorySolution {
    public Angle azimuth;
    public LinearVelocity shooterVelocity;

    public FixedTrajectorySolution(Angle azimuth, LinearVelocity shooterVelocity) {
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
   * @param targetPos 3d displacement of the target, FOC, in meters
   * @param shooterAltitude Angle of the fixed shooter from horizontal
   * @return Time of flight for the projectile from launch to target
   */
  public static FixedTrajectorySolution calcFiringSolution(
      LinearVelocity botVelocityX,
      LinearVelocity botVelocityY,
      Translation3d targetPos,
      Angle shooterAltitude) {
    Time timeOfFlight = calcTargetTime(botVelocityX, botVelocityY, targetPos, shooterAltitude);
    LinearVelocity shooterVelocity = calcShooterVelocity(timeOfFlight, targetPos, shooterAltitude);
    Angle azimuth =
        calcRobotHeadingAzimuth(
            botVelocityX, botVelocityY, timeOfFlight, shooterVelocity, targetPos, shooterAltitude);
    return new FixedTrajectorySolution(azimuth, shooterVelocity);
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
    if (Math.sqrt(
            Math.pow(botVelocityX.in(MetersPerSecond), 2)
                + Math.pow(botVelocityY.in(MetersPerSecond), 2))
        < 1e-2) {
      return 0.0;
    }
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
    List<Double> roots = QuarticSolver.solveQuartic(a, b, c, d);
    if (roots.size() == 0) {
      return 0.0;
    }
    return roots.get(roots.size() - 1);
  }

  public class QuarticSolver {

    private static final double EPSILON = 1e-9;

    /**
     * Solves the quartic equation ax^4 + bx^2 + cx + d = 0 for real roots.
     *
     * @param a Coefficient of x^4
     * @param b Coefficient of x^2
     * @param c Coefficient of x
     * @param d Constant term
     * @return A list of real roots sorted in ascending order.
     */
    public static List<Double> solveQuartic(double a, double b, double c, double d) {
      List<Double> roots = new ArrayList<>();

      // 1. Handle non-quartic case (a = 0)
      if (Math.abs(a) < EPSILON) {
        // It becomes a quadratic: bx^2 + cx + d = 0
        return solveQuadratic(b, c, d);
      }

      // 2. Normalize coefficients: x^4 + Ax^2 + Bx + C = 0
      // Note: Using uppercase A, B, C to distinguish from input params a, b, c
      double A = b / a;
      double B = c / a;
      double C = d / a;

      // 3. Special Case: Biquadratic (B = 0)
      // Equation: x^4 + Ax^2 + C = 0
      // Let u = x^2, then u^2 + Au + C = 0
      if (Math.abs(B) < EPSILON) {
        List<Double> uRoots = solveQuadratic(1.0, A, C);
        for (double u : uRoots) {
          if (u > EPSILON) {
            roots.add(Math.sqrt(u));
            roots.add(-Math.sqrt(u));
          } else if (Math.abs(u) <= EPSILON) {
            roots.add(0.0);
          }
        }
      }
      // 4. General Case: Descartes' Method
      // Factor x^4 + Ax^2 + Bx + C into (x^2 + kx + l)(x^2 - kx + m) = 0
      else {
        // We need to find k^2. Let y = k^2.
        // The resolvent cubic for Descartes method is:
        // y^3 + 2A*y^2 + (A^2 - 4C)y - B^2 = 0

        double c2 = 2 * A;
        double c1 = A * A - 4 * C;
        double c0 = -B * B;

        // Solve the resolvent cubic for a positive real root
        List<Double> cubicRoots = solveCubic(1.0, c2, c1, c0);

        // Theoretically, there is always one positive real root because c0 is negative (-B^2)
        double y = -1.0;
        for (double root : cubicRoots) {
          if (root > EPSILON) {
            y = root;
            break; // Use the first positive root found
          }
        }

        // Should not happen if B != 0, but safety check
        if (y < 0) return roots;

        double k = Math.sqrt(y);

        // Determine l and m
        // l = (A + y - B/k) / 2
        // m = (A + y + B/k) / 2
        double p = A + y;
        double q = B / k;

        double m = (p + q) / 2.0;
        double l = (p - q) / 2.0;

        // Now solve the two resulting quadratic equations:
        // 1) x^2 - kx + m = 0
        roots.addAll(solveQuadratic(1.0, -k, m));

        // 2) x^2 + kx + l = 0
        roots.addAll(solveQuadratic(1.0, k, l));
      }

      Collections.sort(roots);
      return roots;
    }

    /** Helper to solve ax^2 + bx + c = 0 */
    private static List<Double> solveQuadratic(double a, double b, double c) {
      List<Double> roots = new ArrayList<>();
      if (Math.abs(a) < EPSILON) {
        // Linear case: bx + c = 0
        if (Math.abs(b) > EPSILON) {
          roots.add(-c / b);
        }
        return roots;
      }

      double discriminant = b * b - 4 * a * c;

      if (discriminant > EPSILON) {
        double sqrtD = Math.sqrt(discriminant);
        roots.add((-b + sqrtD) / (2 * a));
        roots.add((-b - sqrtD) / (2 * a));
      } else if (Math.abs(discriminant) <= EPSILON) {
        roots.add(-b / (2 * a));
      }
      return roots;
    }

    /**
     * Helper to solve cubic ax^3 + bx^2 + cx + d = 0 Uses the trigonometric solution for stability
     * with real roots.
     */
    private static List<Double> solveCubic(double a, double b, double c, double d) {
      List<Double> roots = new ArrayList<>();
      if (Math.abs(a) < EPSILON) return solveQuadratic(b, c, d);

      // Normalize to x^3 + Ax^2 + Bx + C = 0
      double A = b / a;
      double B = c / a;
      double C = d / a;

      // Depress the cubic: t^3 + pt + q = 0
      // Substitution x = t - A/3
      double A_div_3 = A / 3.0;
      double p = B - A * A_div_3;
      double q = (2 * A * A * A) / 27.0 - (A * B) / 3.0 + C;

      // Discriminant for depressed cubic
      double D = (p / 3.0) * (p / 3.0) * (p / 3.0) + (q / 2.0) * (q / 2.0);

      if (D > EPSILON) {
        // One real root
        double u = Math.cbrt(-q / 2.0 + Math.sqrt(D));
        double v = Math.cbrt(-q / 2.0 - Math.sqrt(D));
        roots.add(u + v - A_div_3);
      } else if (Math.abs(D) <= EPSILON) {
        // Multiple real roots
        if (Math.abs(p) < EPSILON) { // Triple root
          roots.add(-A_div_3);
        } else {
          double t1 = 3 * q / p; // = -2 * (q/2)^(1/3) technically
          double t2 = -1.5 * q / p;
          roots.add(t1 - A_div_3);
          roots.add(t2 - A_div_3);
        }
      } else {
        // Three distinct real roots (Casus irreducibilis)
        double r = Math.sqrt(-p * p * p / 27.0);
        double phi = Math.acos(-q / (2 * r));
        double t = 2 * Math.cbrt(r); // Equivalent to 2 * sqrt(-p/3)

        // Recalculate radius strictly as 2 * sqrt(-p/3) for standard formula
        double radius = 2 * Math.sqrt(-p / 3.0);

        roots.add(radius * Math.cos(phi / 3.0) - A_div_3);
        roots.add(radius * Math.cos((phi + 2 * Math.PI) / 3.0) - A_div_3);
        roots.add(radius * Math.cos((phi + 4 * Math.PI) / 3.0) - A_div_3);
      }
      return roots;
    }
  }
}
