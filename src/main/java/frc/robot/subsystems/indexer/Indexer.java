package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
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
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase implements SysIdSubsystem.SysIdSingleSubsystem {
  public final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private final Alert indexerDisconnectedAlert =
      new Alert("Indexer motor disconnected!", AlertType.kError);

  private final SysIdRoutine sysIdRoutine;

  public Indexer(IndexerIO io) {
    this.io = io;
    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Velocity.ofBaseUnits(0.2, Volts.per(Second)),
                Voltage.ofBaseUnits(2, Volts),
                Second.of(15),
                (state) -> Logger.recordOutput("Indexer/SysIdState", state.toString())),
            new Mechanism(io::indexerOpenLoop, null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    indexerDisconnectedAlert.set(!inputs.indexerConnected && Constants.CURRENT_MODE != Mode.SIM);
  }

  public Command setIndexerVelocity(Supplier<AngularVelocity> velocity) {
    return new InstantCommand(
            () -> {
              io.setIndexerVelocity(velocity.get());
            },
            this)
        .withName("Set Indexer Velocity");
  }

  @Override
  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }

  @Override
  public String getName() {
    return "Indexer ";
  }

  @Override
  public Command reset(Direction direction) {
    return Commands.sequence(
        Commands.runOnce(() -> io.indexerOpenLoop(Volts.of(0)), this),
        Commands.idle(this)
            .until(
                () ->
                    inputs.indexerVelocityRadPerSec.isNear(
                        RotationsPerSecond.of(0), IndexerConstants.RESET_TOLERANCE)),
        Commands.runOnce(() -> io.indexerOpenLoop(Volts.of(0)), this));
  }
}
