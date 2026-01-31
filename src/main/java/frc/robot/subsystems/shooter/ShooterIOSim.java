package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.sim.SimMechanism;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSim extends ShooterIOSpark implements SimMechanism {

  FlywheelSim shooterPhysicsSim;

  SparkFlexSim motorSim;
  SparkRelativeEncoderSim encoderSim;

  int shooterNum;

  public ShooterIOSim(int shooterNum) {
    super(shooterNum);

    this.shooterNum = shooterNum;

    DCMotor gearbox = DCMotor.getNeoVortex(1);

    shooterPhysicsSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeoVortex(1),
                ShooterConstants.ShooterSimConstants.SHOOTER_MOI.in(KilogramSquareMeters),
                ShooterConstants.GEARING),
            gearbox);
    motorSim = new SparkFlexSim(shooterMotor, gearbox);
    encoderSim = new SparkRelativeEncoderSim(shooterMotor);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    shooterPhysicsSim.setInputVoltage(
        motorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    shooterPhysicsSim.update(0.02);

    motorSim.iterate(
        shooterPhysicsSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);

    encoderSim.iterate(shooterPhysicsSim.getAngularVelocityRPM(), 0.02);

    encoderSim.setVelocity(shooterPhysicsSim.getAngularVelocityRPM());

    Logger.recordOutput(
        "Shooter" + shooterNum + "/ShooterVelocity", shooterPhysicsSim.getAngularVelocity());
    Logger.recordOutput(
        "Shooter" + shooterNum + "/ShooterAppliedVolts", shooterPhysicsSim.getInputVoltage());
    Logger.recordOutput(
        "Shooter" + shooterNum + "/ShooterCurrentAmps", shooterPhysicsSim.getCurrentDrawAmps());

    super.updateInputs(inputs);
  }

  @Override
  public double[] getCurrents() {
    return new double[] {shooterPhysicsSim.getCurrentDrawAmps()};
  }
}
