package frc.robot.subsystems.groundintake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.SysIdSubsystem;
import frc.robot.subsystems.groundintake.GroundIntakeConstants.GroundIntakePivotConstants;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class GroundIntake extends SubsystemBase implements SysIdSubsystem {

  private final GroundIntakeRollerIO ioRoller;
  private final GroundIntakePivotIO ioPivot;
  public final GroundIntakeRollerIOInputsAutoLogged inputsRoller =
      new GroundIntakeRollerIOInputsAutoLogged();
  public final GroundIntakePivotIOInputsAutoLogged inputsPivot =
      new GroundIntakePivotIOInputsAutoLogged();

  private final Alert groundIntakeRollerDisconnectedAlert =
      new Alert("GroundIntakeRoller motor disconnected!!", AlertType.kError);
  private final Alert groundIntakePivotDisconnectedAlert =
      new Alert("GroundIntakePivot motor disconnected!!", AlertType.kError);

  private final SysIdRoutine sysIdRoutineRoller;
  private final SysIdRoutine sysIdRoutinePivot;

  public GroundIntake(GroundIntakeRollerIO ioRoller, GroundIntakePivotIO ioPivot) {
    this.ioRoller = ioRoller;
    this.ioPivot = ioPivot;
    this.sysIdRoutineRoller =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Velocity.ofBaseUnits(0.2, Volts.per(Second)),
                Voltage.ofBaseUnits(2, Volts),
                Second.of(30),
                state -> Logger.recordOutput("GroundIntakeRoller/SysIdState", state.toString())),
            new Mechanism(ioRoller::setGroundIntakeRollerOpenLoop, null, this));
    this.sysIdRoutinePivot =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Velocity.ofBaseUnits(0.2, Volts.per(Second)),
                Voltage.ofBaseUnits(2, Volts),
                Second.of(30),
                state -> Logger.recordOutput("GroundIntakePivot/SysIdState", state.toString())),
            new Mechanism(ioPivot::setGroundIntakePivotOpenLoop, null, this));
  }

  @Override
  public void periodic() {
    ioRoller.updateInputs(inputsRoller);
    ioPivot.updateInputs(inputsPivot);
    Logger.processInputs("GroundIntakeRoller", inputsRoller);
    Logger.processInputs("GroundIntakePivot", inputsPivot);

    groundIntakeRollerDisconnectedAlert.set(
        !inputsRoller.groundIntakeRollerConnected && Constants.CURRENT_MODE != Mode.SIM);
    groundIntakePivotDisconnectedAlert.set(
        !inputsPivot.groundIntakePivotConnected && Constants.CURRENT_MODE != Mode.SIM);
  }

  public Command setGroundIntakeRollerVelocity(Supplier<AngularVelocity> velocity) {
    return new InstantCommand(() -> ioRoller.setGroundIntakeRollerVelocity(velocity.get()), this);
  }

  public Command setGroundIntakePivotPosition(Angle position) {
    return new StartEndCommand(
            () -> ioPivot.setGroundIntakePivotPosition(position),
            () -> ioPivot.setGroundIntakePivotVelocity(RadiansPerSecond.of(0)),
            this)
        .until(
            () ->
                position.isNear(
                    inputsPivot.groundIntakePivotPositionRads,
                    GroundIntakeConstants.GroundIntakePivotConstants.PIVOT_POSITION_TOLERANCE));
  }

  public Command setGroundIntakePivotVelocity(Supplier<AngularVelocity> velocity) {
    return new InstantCommand(() -> ioPivot.setGroundIntakePivotVelocity(velocity.get()), this);
  }

  public Command setGroundIntakePivotVoltage(Supplier<Voltage> voltage) {
    return new InstantCommand(() -> ioPivot.setGroundIntakePivotOpenLoop(voltage.get()), this);
  }

  public List<SysIdTarget> getSysIdTargets() {
    return List.of(
        new SysIdTarget("GroundIntakeRoller", sysIdRoutineRoller, this::rollerReset),
        new SysIdTarget(
            "GroundIntakePivot",
            sysIdRoutinePivot,
            this::pivotReset,
            this::shouldPivotSafetyEscape));
  }

  private Boolean shouldPivotSafetyEscape(Direction direction) {
    return inputsPivot.groundIntakePivotPositionRads.isNear(
        (direction == Direction.kForward)
            ? GroundIntakePivotConstants.MAX_ANGLE
            : GroundIntakePivotConstants.MIN_ANGLE,
        GroundIntakeConstants.GroundIntakePivotConstants.SAFETY_ESCAPE_TOLERANCE);
  }

  private Command pivotReset(Direction direction) {
    return Commands.sequence(
        Commands.runOnce(() -> ioPivot.setGroundIntakePivotOpenLoop(Volts.of(0)), this),
        Commands.idle(this)
            .until(
                () ->
                    inputsPivot.groundIntakePivotVelocityRadsPerSec.isNear(
                        RotationsPerSecond.of(0),
                        GroundIntakeConstants.GroundIntakePivotConstants.VELOCITY_RESET_TOLERANCE)),
        new ConditionalCommand(
            Commands.runOnce(() -> ioPivot.setGroundIntakePivotOpenLoop(Volts.of(-2)), this)
                .andThen(
                    Commands.idle(this)
                        .until(
                            () ->
                                inputsPivot.groundIntakePivotPositionRads.isNear(
                                    GroundIntakePivotConstants.MIN_ANGLE,
                                    GroundIntakeConstants.GroundIntakePivotConstants
                                        .PIVOT_POSITION_TOLERANCE))),
            Commands.runOnce(() -> ioPivot.setGroundIntakePivotOpenLoop(Volts.of(2)), this)
                .andThen(
                    Commands.idle(this)
                        .until(
                            () ->
                                inputsPivot.groundIntakePivotPositionRads.isNear(
                                    GroundIntakePivotConstants.MAX_ANGLE,
                                    GroundIntakeConstants.GroundIntakePivotConstants
                                        .PIVOT_POSITION_TOLERANCE))),
            () -> direction == Direction.kForward),
        Commands.runOnce(() -> ioPivot.setGroundIntakePivotOpenLoop(Volts.of(0)), this),
        Commands.idle(this)
            .until(
                () ->
                    inputsPivot.groundIntakePivotVelocityRadsPerSec.isNear(
                        RotationsPerSecond.of(0),
                        GroundIntakeConstants.GroundIntakePivotConstants
                            .VELOCITY_RESET_TOLERANCE)));
  }

  private Command rollerReset(Direction direction) {
    return Commands.sequence(
        Commands.runOnce(() -> ioRoller.setGroundIntakeRollerOpenLoop(Volts.of(0)), this),
        Commands.idle(this)
            .until(
                () ->
                    inputsRoller.groundIntakeRollerVelocityRadsPerSec.isNear(
                        RotationsPerSecond.of(0),
                        GroundIntakeConstants.GroundIntakeRollerConstants.RESET_TOLERANCE)),
        Commands.runOnce(() -> ioRoller.setGroundIntakeRollerOpenLoop(Volts.of(0)), this));
  }
}
