package frc.robot.util;

import static edu.wpi.first.units.Units.*;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class ProjectileTrajectoryUtils {
  public class AirResistanceSolver {
    public static double g = GRAVITY.in(MetersPerSecondPerSecond);

    public static Matrix<N2, N2> calcJacobian(
        Matrix<N2, N1> in, double v_x, double v_y, double z, double alpha) {
      double v_s = in.get(0, 0);
      double theta = in.get(1, 0);
      Matrix<N2, N2> J =
          new Matrix<>(
              Nat.N2(),
              Nat.N2(),
              new double[] {
                (v_x * sin(alpha) / g)
                    + ((v_x * pow(sin(alpha), 2) * v_s)
                        / (g * sqrt(pow(sin(alpha), 2) * pow(v_s, 2) - 2 * g * z)))
                    + 2 * cos(alpha) * cos(theta) * sin(alpha) * v_s / g
                    + (cos(alpha) * cos(theta) / g)
                        * (pow(sin(alpha), 2) * pow(v_s, 3) - 2 * g * z * v_s)
                        / (sqrt(pow(sin(alpha), 2) * pow(v_s, 4) - 2 * g * z * pow(v_s, 2))),
                ((v_s * sin(alpha) + sqrt(pow(v_s, 2) * pow(sin(alpha), 2) - 2 * g * z)) / g)
                    * cos(alpha)
                    * v_s
                    * (-sin(theta)),
                (v_y * sin(alpha) / g)
                    + ((v_y * pow(sin(alpha), 2) * v_s)
                        / (g * sqrt(pow(sin(alpha), 2) * pow(v_s, 2) - 2 * g * z)))
                    + 2 * cos(alpha) * sin(theta) * sin(alpha) * v_s / g
                    + (cos(alpha) * sin(theta) / g)
                        * (pow(sin(alpha), 2) * pow(v_s, 3) - 2 * g * z * v_s)
                        / (sqrt(pow(sin(alpha), 2) * pow(v_s, 4) - 2 * g * z * pow(v_s, 2))),
                ((v_s * sin(alpha) + sqrt(pow(v_s, 2) * pow(sin(alpha), 2) - 2 * g * z)) / g)
                    * cos(alpha)
                    * v_s
                    * (cos(theta))
              });
      return J;
    }

    public static double dragAccel(double vMag) {
      double Cd = 0.47; // Drag coefficient for a sphere / fuel
      Cd = 0.75; // Drag coefficient for a lunite
      double A =
          0.15 * 0.15 / 4 * 3.14159265358979323846; // Cross-sectional area in m^2 of a Fuel piece
      A = 0.0072; // Approximate cross-sectional area of a lunite
      double rho = 1.225; // Air density in kg/m^3
      double m = 0.23; // Mass in kg of a Fuel piece
      m = 0.069; // Mass in kg of a lunite
      double F_d = 0.5 * Cd * rho * A * vMag * vMag; // Drag force
      double a_d = F_d / m; // Acceleration due to drag
      return a_d;
    }

    public static Matrix<N2, N1> f_airResistance(
        Matrix<N2, N1> in, double v_x, double v_y, double z, double alpha) {
      double v_s = in.get(0, 0);
      double theta = in.get(1, 0);
      double xPos = 0;
      double yPos = 0;
      double zPos = 0;
      double xVel = v_x + v_s * cos(theta) * cos(alpha);
      double yVel = v_y + v_s * sin(theta) * cos(alpha);
      double zVel = v_s * sin(alpha);
      int simSteps = 970;
      for (int i = 0; i < simSteps; i++) {
        double t = ((i + 1.0) / simSteps) * 4.0; // Simulate up to 4 seconds
        xPos += xVel * (4.0 / simSteps);
        yPos += yVel * (4.0 / simSteps);
        zPos += zVel * (4.0 / simSteps);

        double vMag = sqrt(xVel * xVel + yVel * yVel + zVel * zVel);
        double a_d = dragAccel(vMag);
        xVel -= (a_d * (xVel / vMag)) * (4.0 / simSteps);
        yVel -= (a_d * (yVel / vMag)) * (4.0 / simSteps);
        zVel -= (a_d * (zVel / vMag)) * (4.0 / simSteps);
        zVel -= g * (4.0 / simSteps); // Gravity effect

        if (zVel < 0.0 && zPos < z) {
          return new Matrix<>(Nat.N2(), Nat.N1(), new double[] {xPos, yPos});
        }
      }
      return new Matrix<>(Nat.N2(), Nat.N1(), new double[] {0.0, 0.0});
    }

    public static class State {
      double x, y, z;
      double vx, vy, vz;

      public State() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        this.vx = 0;
        this.vy = 0;
        this.vz = 0;
      }

      public State(double x, double y, double z, double vx, double vy, double vz) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.vx = vx;
        this.vy = vy;
        this.vz = vz;
      }
    }

    /** Derivative function: returns dS/dt */
    public static State get_derivative(State s) {
      double speed = Math.sqrt(s.vx * s.vx + s.vy * s.vy + s.vz * s.vz);

      State deriv = new State();
      deriv.x = s.vx;
      deriv.y = s.vy;
      deriv.z = s.vz;

      // Acceleration: Gravity + Drag (opposite to velocity direction)
      dragAccel(speed);
      deriv.vx = -(dragAccel(speed) * (s.vx / speed));
      deriv.vy = -(dragAccel(speed) * (s.vy / speed));
      deriv.vz = -g - (dragAccel(speed) * (s.vz / speed));

      return deriv;
    }

    // RK4 Step function
    public static State rk4_step(State s, double dt) {
      State k1, k2, k3, k4, next_s;

      // k1 = f(t, s)
      k1 = get_derivative(s);

      // k2 = f(t + dt/2, s + k1 * dt/2)
      State s2 =
          new State(
              s.x + k1.x * dt / 2,
              s.y + k1.y * dt / 2,
              s.z + k1.z * dt / 2,
              s.vx + k1.vx * dt / 2,
              s.vy + k1.vy * dt / 2,
              s.vz + k1.vz * dt / 2);
      k2 = get_derivative(s2);

      // k3 = f(t + dt/2, s + k2 * dt/2)
      State s3 =
          new State(
              s.x + k2.x * dt / 2,
              s.y + k2.y * dt / 2,
              s.z + k2.z * dt / 2,
              s.vx + k2.vx * dt / 2,
              s.vy + k2.vy * dt / 2,
              s.vz + k2.vz * dt / 2);
      k3 = get_derivative(s3);

      // k4 = f(t + dt, s + k3 * dt)
      State s4 =
          new State(
              s.x + k3.x * dt,
              s.y + k3.y * dt,
              s.z + k3.z * dt,
              s.vx + k3.vx * dt,
              s.vy + k3.vy * dt,
              s.vz + k3.vz * dt);
      k4 = get_derivative(s4);

      next_s = new State();
      // Combine steps: s_next = s + (dt/6) * (k1 + 2k2 + 2k3 + k4)
      next_s.x = s.x + (dt / 6.0) * (k1.x + 2 * k2.x + 2 * k3.x + k4.x);
      next_s.y = s.y + (dt / 6.0) * (k1.y + 2 * k2.y + 2 * k3.y + k4.y);
      next_s.z = s.z + (dt / 6.0) * (k1.z + 2 * k2.z + 2 * k3.z + k4.z);
      next_s.vx = s.vx + (dt / 6.0) * (k1.vx + 2 * k2.vx + 2 * k3.vx + k4.vx);
      next_s.vy = s.vy + (dt / 6.0) * (k1.vy + 2 * k2.vy + 2 * k3.vy + k4.vy);
      next_s.vz = s.vz + (dt / 6.0) * (k1.vz + 2 * k2.vz + 2 * k3.vz + k4.vz);

      return next_s;
    }

    public static final int maxItersBisection = 15;
    public static final double toleranceBisect = 1e-3;

    public static Matrix<N2, N1> bisectionMethodRK4(State low, State high, double dt, double z) {
      for (int i = 0; i < maxItersBisection; i++) {
        dt /= 2.0;
        State mid = rk4_step(low, dt);
        if (mid.z > z) {
          low = mid;
        } else {
          high = mid;
        }
        // if (low.z - high.z < toleranceBisect) {
        //   break;
        // }
      }
      return new Matrix<>(Nat.N2(), Nat.N1(), new double[] {low.x, low.y});
    }

    public static final int simStepsRK4 = 50;

    public static Matrix<N2, N1> f_airResistance_RK4(
        Matrix<N2, N1> in, double v_x, double v_y, double z, double alpha) {
      double v_s = in.get(0, 0);
      double theta = in.get(1, 0);
      State state = new State();
      state.x = 0;
      state.y = 0;
      state.z = 0;
      state.vx = v_x + v_s * cos(theta) * cos(alpha);
      state.vy = v_y + v_s * sin(theta) * cos(alpha);
      state.vz = v_s * sin(alpha);

      double totalTime = 4.0; // Simulate up to 4 seconds
      double dt = totalTime / simStepsRK4;

      for (int i = 0; i < simStepsRK4; i++) {
        State nextState = rk4_step(state, dt);

        if (nextState.vz < 0 && nextState.z < z) {
          return bisectionMethodRK4(state, nextState, dt, z);
        }

        state = nextState;
      }
      return new Matrix<>(Nat.N2(), Nat.N1(), new double[] {0, 0});
    }

    public static double dist(Matrix<N2, N1> v) {
      return sqrt(pow(v.get(0, 0), 2) + pow(v.get(1, 0), 2));
    }

    private static final int MAX_ITERS = 4;
    private static final double tolerance = 0.001;

    public static class TrajectorySolution {
      public LinearVelocity shooterVelocity;
      public Angle azimuth;

      public TrajectorySolution(LinearVelocity shooterVelocity, Angle azimuth) {
        this.shooterVelocity = shooterVelocity;
        this.azimuth = azimuth;
      }

      public TrajectorySolution(Matrix<N2, N1> vector) {
        this.shooterVelocity = MetersPerSecond.of(vector.get(0, 0));
        this.azimuth = Radians.of(vector.get(1, 0));
      }
    }

    public static TrajectorySolution newtonRhapsonSolveAirResistance(
        LinearVelocity botVelocityX,
        LinearVelocity botVelocityY,
        Translation3d targetPos,
        Angle shooterAltitude,
        FixedTrajectorySolution guessSolution) {
      double start = HALUtil.getFPGATime();
      if (guessSolution == null) {
        guessSolution = calcFiringSolution(botVelocityX, botVelocityY, targetPos, shooterAltitude);
      }
      Matrix<N2, N1> guess =
          new Matrix<>(
              Nat.N2(),
              Nat.N1(),
              new double[] {
                guessSolution.shooterVelocity.in(MetersPerSecond), guessSolution.azimuth.in(Radians)
              });
      Matrix<N2, N1> x = guess;
      try {
        for (int count = 0; count < MAX_ITERS; count++) {
          var startRk4 = HALUtil.getFPGATime();
          Matrix<N2, N1> f_x =
              f_airResistance_RK4(
                      x,
                      botVelocityX.in(MetersPerSecond),
                      botVelocityY.in(MetersPerSecond),
                      targetPos.getZ(),
                      shooterAltitude.in(Radians))
                  .minus(
                      new Matrix<>(
                          Nat.N2(), Nat.N1(), new double[] {targetPos.getX(), targetPos.getY()}));
          Logger.recordOutput("trajectoryCalc/rk4Time" + count, (HALUtil.getFPGATime() - startRk4));
          if (count % 1 == 0) {
            // System.out.println("\niteration: " + (count + 1) + "  f_x:" + dist(f_x) + "\n");
            // System.out.println(x);
            // System.out.println("\n");
            // System.out.println(f_x);
            // System.out.println("\n");
          }
          // if (dist(f_x) < tolerance) return new TrajectorySolution(x);
          startRk4 = HALUtil.getFPGATime();
          Matrix<N2, N2> J =
              calcJacobian(
                  x,
                  botVelocityX.in(MetersPerSecond),
                  botVelocityY.in(MetersPerSecond),
                  targetPos.getZ(),
                  shooterAltitude.in(Radians));
          Logger.recordOutput(
              "trajectoryCalc/jacobian" + count, (HALUtil.getFPGATime() - startRk4));
          x = x.minus(J.inv().times(f_x));

          if (HALUtil.getFPGATime() - start > 0.005 * 1e6) {
            throw new Exception("Timed out");
          }
        }
        return new TrajectorySolution(x);
      } catch (Exception e) {
        return new TrajectorySolution(guess);
      }
    }
  }

  private static final int simSteps = 100;

  public static Distance minDistTrajectory(
      LinearVelocity botVelocityX,
      LinearVelocity botVelocityY,
      Translation3d targetPos,
      Angle shooterAltitude,
      Angle azimuth,
      LinearVelocity shooterVelocity) {
    Logger.recordOutput("minDistTrajectory/velX", botVelocityX);
    Logger.recordOutput("minDistTrajectory/velY", botVelocityY);
    Logger.recordOutput("minDistTrajectory/shooterVelocity", shooterVelocity);
    Logger.recordOutput("minDistTrajectory/shooterAltitude", shooterAltitude);
    Logger.recordOutput("minDistTrajectory/azimuth", azimuth);
    Logger.recordOutput("minDistTrajectory/targetX", Meters.of(targetPos.getX()));
    Logger.recordOutput("minDistTrajectory/targetY", Meters.of(targetPos.getY()));
    Logger.recordOutput("minDistTrajectory/targetZ", Meters.of(targetPos.getZ()));
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

    double minDist = 1;
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
      if (dist < minDist && Math.abs(targetPos.getZ() - z) < 0.1) {
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
   * @param botVelocityX X-velocity of the Robot, FOC, in meters per second
   * @param botVelocityY Y-velocity of the Robot, FOC, in meters per second
   * @param botAccelX X-velocity of the Robot, FOC, in meters per second per second
   * @param botAccelY Y-velocity of the Robot, FOC, in meters per second per second
   * @param targetPos 3d displacement of the target, FOC, in meters
   * @param shooterAltitude Angle of the fixed shooter from horizontal
   * @return Time of flight for the projectile from launch to target
   */
  public static MovingTrajectorySolution calcMovingFiringSolution(
      LinearVelocity botVelocityX,
      LinearVelocity botVelocityY,
      LinearAcceleration botAccelX,
      LinearAcceleration botAccelY,
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
    double dfdt =
        calcDfdt(botVelocityX, botVelocityY, botAccelX, botAccelY, targetPos, shooterAltitude);
    Logger.recordOutput("MovingTrajectory/dfdt", dfdt);
    LinearVelocity shooterVelocity = calcShooterVelocity(timeOfFlight, targetPos, shooterAltitude);
    Logger.recordOutput("MovingTrajectory/shooterVelocity", shooterVelocity);
    LinearAcceleration shooterAcceleration =
        calcShooterAcceleration(timeOfFlight, dfdt, targetPos, shooterAltitude);
    Logger.recordOutput("MovingTrajectory/shooterAcceleration", shooterAcceleration);
    Angle azimuth =
        calcRobotHeadingAzimuth(
            botVelocityX, botVelocityY, timeOfFlight, targetPos, shooterAltitude);
    Logger.recordOutput("MovingTrajectory/azimuth", azimuth);
    AngularVelocity omega =
        calcRobotAngularVelocity(
            botVelocityX,
            botVelocityY,
            botAccelX,
            botAccelY,
            timeOfFlight,
            dfdt,
            targetPos,
            shooterAltitude);
    Logger.recordOutput("MovingTrajectory/omega", omega);
    return new MovingTrajectorySolution(azimuth, omega, shooterVelocity, shooterAcceleration);
  }

  /**
   * Calculates the Robot moving firing solution ASSUMING CONSTANT VELOCITY
   *
   * @param botVelocityX X-velocity of the Robot, FOC, in meters
   * @param botVelocityY Y-velocity of the Robot, FOC, in meters
   * @param targetPos 3d displacement of the target, FOC, in meters
   * @param shooterAltitude Angle of the fixed shooter from horizontal
   * @return Time of flight for the projectile from launch to target
   */
  public static MovingTrajectorySolution calcMovingFiringSolutionConstantVelocity(
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
    double dfdt =
        calcDfdt(
            botVelocityX,
            botVelocityY,
            MetersPerSecondPerSecond.of(0),
            MetersPerSecondPerSecond.of(0),
            targetPos,
            shooterAltitude);
    Logger.recordOutput("MovingTrajectory/dfdt", dfdt);
    LinearVelocity shooterVelocity = calcShooterVelocity(timeOfFlight, targetPos, shooterAltitude);
    Logger.recordOutput("MovingTrajectory/shooterVelocity", shooterVelocity);
    LinearAcceleration shooterAcceleration =
        calcShooterAcceleration(timeOfFlight, dfdt, targetPos, shooterAltitude);
    Logger.recordOutput("MovingTrajectory/shooterAcceleration", shooterAcceleration);
    Angle azimuth =
        calcRobotHeadingAzimuth(
            botVelocityX, botVelocityY, timeOfFlight, targetPos, shooterAltitude);
    Logger.recordOutput("MovingTrajectory/azimuth", azimuth);
    AngularVelocity omega =
        calcRobotAngularVelocity(
            botVelocityX,
            botVelocityY,
            MetersPerSecondPerSecond.of(0),
            MetersPerSecondPerSecond.of(0),
            timeOfFlight,
            dfdt,
            targetPos,
            shooterAltitude);
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
            botVelocityX, botVelocityY, timeOfFlight, targetPos, shooterAltitude);
    return new FixedTrajectorySolution(azimuth, shooterVelocity);
  }

  /**
   * Calculates the Robot heading azimuth
   *
   * @param botVelocityX X-velocity of the Robot, FOC, in meters
   * @param botVelocityY Y-velocity of the Robot, FOC, in meters
   * @param timeOfFlight duration of the delta-t from launch to target
   * @param targetPos 3d displacement of the target, FOC, in meters
   * @param shooterAltitude Angle of the fixed shooter from horizontal
   * @return Time of flight for the projectile from launch to target
   */
  public static Angle calcRobotHeadingAzimuth(
      LinearVelocity botVelocityX,
      LinearVelocity botVelocityY,
      Time timeOfFlight,
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
      LinearAcceleration botAccelX,
      LinearAcceleration botAccelY,
      Time timeOfFlight,
      double dfdt,
      Translation3d targetPos,
      Angle shooterAltitude) {
    double deltaY = targetPos.getY() - timeOfFlight.in(Seconds) * botVelocityY.in(MetersPerSecond);
    double deltaX = targetPos.getX() - timeOfFlight.in(Seconds) * botVelocityX.in(MetersPerSecond);
    double vX = botVelocityX.in(MetersPerSecond);
    double vY = botVelocityY.in(MetersPerSecond);
    double aX = botAccelX.in(MetersPerSecondPerSecond);
    double aY = botAccelY.in(MetersPerSecondPerSecond);
    double tF = timeOfFlight.in(Seconds);
    double numTerm1 = (vY - dfdt * vY - tF * aY) * deltaX;
    double numTerm2 = (vX - dfdt * vX - tF * aX) * deltaY;
    double denominator = Math.pow(deltaX, 2) + Math.pow(deltaY, 2);
    AngularVelocity omega = RadiansPerSecond.of((numTerm1 - numTerm2) / denominator);
    // return omega.times(-1); // idk why this is necessary, could be field orientation
    double h = 1e-4;
    Angle azimuth1 = Radians.of(Math.atan2(deltaY, deltaX));
    double deltaY2 = (targetPos.getY() - vY * h) - (tF + dfdt * h) * (vY + aY * h);
    double deltaX2 = (targetPos.getX() - vX * h) - (tF + dfdt * h) * (vX + aX * h);
    Angle azimuth2 = Radians.of(Math.atan2(deltaY2, deltaX2));
    return RadiansPerSecond.of((azimuth2.in(Radians) - azimuth1.in(Radians)) / h);
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
      LinearAcceleration botAccelX,
      LinearAcceleration botAccelY,
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
    double a_x = botAccelX.in(MetersPerSecondPerSecond);
    double a_y = botAccelY.in(MetersPerSecondPerSecond);
    double a = 0.25 * Math.pow(g, 2) * cos2a;
    double b = g * targetPos.getZ() * cos2a - v_x * v_x * sin2a - v_y * v_y * sin2a;
    double c = 2.0 * (x * v_x + y * v_y) * sin2a;
    double d = Math.pow(targetPos.getZ(), 2) * cos2a - x * x * sin2a - y * y * sin2a;

    double bPrime = -2 * sin2a * (v_x * a_x + v_y * a_y);
    double cPrime = 2 * sin2a * (v_x * v_x + a_x * x + v_y * v_y + a_y * y);
    double dPrime = -2 * sin2a * (x * v_x + y * v_y);
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
