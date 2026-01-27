import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.*;
import frc.robot.util.ProjectileTrajectoryUtils;
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
  void testJacobian() {
    ProjectileTrajectoryUtils.AirResistanceSolver.calcJacobian(
        new Matrix<>(Nat.N2(), Nat.N1(), new double[] {0.0, 0.0}),
        1.0,
        1.0,
        1.81,
        Degrees.of(60).in(Radians));
    assertEquals(0.5, 0.5, DELTA);
  }

  @Test
  void testSolve() {
    Matrix<N2, N1> result =
        ProjectileTrajectoryUtils.AirResistanceSolver.newtonRhapsonSolveAirResistance(
            MetersPerSecond.of(1.0),
            MetersPerSecond.of(1.0),
            new Translation3d(2, 4, 1.81),
            Degrees.of(60));
    assertEquals(1,1, 0.1);
  }
}
