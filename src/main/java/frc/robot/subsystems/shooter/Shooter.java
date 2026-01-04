package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.SysIdSubsystem;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase implements SysIdSubsystem {

  public final ShooterIO io1, io2;
  public final ShooterIOInputsAutoLogged inputs1 = new ShooterIOInputsAutoLogged(),
      inputs2 = new ShooterIOInputsAutoLogged();
  private final SysIdRoutine sysIdRoutine1;
  private final SysIdRoutine sysIdRoutine2;

  private final Alert ShooterDisconnectedAlert =
      new Alert("Shooter motor disconnected", AlertType.kWarning);

  public Shooter(ShooterIO io1, ShooterIO io2) {
    super();
    this.io1 = io1;
    this.io2 = io2;
    this.sysIdRoutine1 =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new Mechanism(io1::shooterOpenLoop, null, this));
    this.sysIdRoutine2 =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new Mechanism(io2::shooterOpenLoop, null, this));
  }

  @Override
  public void periodic() {
    io1.updateInputs(inputs1);
    Logger.processInputs("Shooter1", inputs1);

    ShooterDisconnectedAlert.set(!inputs1.shooterConnected && Constants.CURRENT_MODE != Mode.SIM);

    io2.updateInputs(inputs2);
    Logger.processInputs("Shooter2", inputs2);

    ShooterDisconnectedAlert.set(!inputs2.shooterConnected && Constants.CURRENT_MODE != Mode.SIM);
  }

  /**
   * Sets the shooter to the given velocity in degrees per second.
   *
   * <p>This is a non-blocking call and will not wait until the shooter is at the requested
   * velocity.
   *
   * @param velocity The velocity to set the shooter to in degrees per second.
   * @return A command that sets the shooter to the given velocity.
   */
  public Command setShooterVelocity(Supplier<AngularVelocity> velocity) {
    return new InstantCommand(
            () -> {
              io1.setShooterVelocity(velocity.get());
              io2.setShooterVelocity(velocity.get());
            },
            this)
        .withName("Set Shooter Velocity");
  }

  public Command setShooterVelocity(
      Supplier<AngularVelocity> velocity1, Supplier<AngularVelocity> velocity2) {
    return new InstantCommand(
            () -> {
              io1.setShooterVelocity(velocity1.get());
              io2.setShooterVelocity(velocity2.get());
            },
            this)
        .withName("Set Shooter Velocity(Dual)");
  }

  public Command reset(Direction direction) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  io1.shooterOpenLoop(Volts.of(0));
                  io2.shooterOpenLoop(Volts.of(0));
                },
                this),
            Commands.idle(this)
                .until(
                    () ->
                        inputs1.shooterVelocityRadPerSec.isNear(
                                RotationsPerSecond.ofBaseUnits(0), ShooterConstants.RESET_TOLERANCE)
                            && inputs2.shooterVelocityRadPerSec.isNear(
                                RotationsPerSecond.ofBaseUnits(0),
                                ShooterConstants.RESET_TOLERANCE)),
            Commands.runOnce(
                () -> {
                  io1.shooterOpenLoop(Volts.of(0));
                  io2.shooterOpenLoop(Volts.of(0));
                },
                this))
        .withName("Reset Shooter");
  }

  @Override
  public List<SysIdTarget> getSysIdTargets() {
    return List.of(
        new SysIdTarget("Shooter ", sysIdRoutine1, this::reset),
        new SysIdTarget("Shooter2 ", sysIdRoutine2, this::reset));
  }
}
