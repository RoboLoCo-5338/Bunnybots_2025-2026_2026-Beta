import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.nodes.jni.nodesJNI;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.*;
import frc.robot.util.ProjectileTrajectoryUtils;
import frc.robot.util.ProjectileTrajectoryUtils.AirResistanceSolver.TrajectorySolution;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class NewtonSolverTest {
  static final double DELTA = 1e-2; // acceptable deviation range

  @BeforeEach // this method will run before each test
  void setup() {}

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {}

  @Test
  void testSolve() {
    nodesJNI.configureParameters(0.75, 0.0072, 0.069, 1.225);
    for (int i = 0; i < 10; i++) {
      runSingleSolve();
    }
  }

  void runSingleSolve() {
    double vX = Math.signum(Math.random() - 0.5) * (Math.random() * 6 - 3);
    double vY = Math.signum(Math.random() - 0.5) * (Math.random() * 6 - 3);

    double x = Math.signum(Math.random() - 0.5) * (Math.random() * 4 + 3);
    double y = Math.signum(Math.random() - 0.5) * (Math.random() * 4 + 3);

    TrajectorySolution result =
        ProjectileTrajectoryUtils.AirResistanceSolver.newtonRhapsonSolveAirResistance(
            MetersPerSecond.of(vX),
            MetersPerSecond.of(vY),
            new Translation3d(x, y, 1.81),
            Degrees.of(50),
            null);
    nodesJNI.TrajectorySolution jniResult =
        nodesJNI.newtonRhapsonSolveAirResistance(vX, vY, x, y, 1.81, Degrees.of(50).in(Radians));

    System.out.println(
        "Java Result: "
            + result.shooterVelocity.in(MetersPerSecond)
            + " m/s, "
            + result.azimuth.in(Degrees)
            + " degrees");
    System.out.println(
        "JNI Result: "
            + jniResult.shooterVel
            + " m/s, "
            + jniResult.azimuth * (180.0 / Math.PI)
            + " degrees");

    assertEquals(result.shooterVelocity.in(MetersPerSecond), jniResult.shooterVel, 0.01);
    assertEquals(result.azimuth.in(Radians), jniResult.azimuth, 0.01);
  }
}
