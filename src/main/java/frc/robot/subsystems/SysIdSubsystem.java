package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.List;
import java.util.function.Function;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public interface SysIdSubsystem {
  public static record SysIdTarget(
      String name,
      SysIdRoutine routine,
      Function<SysIdRoutine.Direction, Command> reset,
      Function<SysIdRoutine.Direction, Boolean> safetyEscapeSupplier) {
    public SysIdTarget(
        String name, SysIdRoutine routine, Function<SysIdRoutine.Direction, Command> reset) {
      this(name, routine, reset, safetyEscapeSupplier -> false);
    }
  }

  public List<SysIdTarget> getSysIdTargets();

  public default void addRoutinesToChooser(LoggedDashboardChooser<Command> autoChooser) {
    for (SysIdTarget target : getSysIdTargets()) {
      addSysIdOptions(autoChooser, target);
    }
  }

  private void addSysIdOptions(LoggedDashboardChooser<Command> autoChooser, SysIdTarget target) {
    autoChooser.addOption(
        target.name + "SysId Quasistatic Forward",
        target
            .routine
            .quasistatic(SysIdRoutine.Direction.kForward)
            .until(() -> target.safetyEscapeSupplier.apply(Direction.kForward))
            .andThen(target.reset.apply(Direction.kForward).withTimeout(5)));
    autoChooser.addOption(
        target.name + "SysId Quasistatic Backward",
        target
            .routine
            .quasistatic(SysIdRoutine.Direction.kReverse)
            .until(() -> target.safetyEscapeSupplier.apply(Direction.kReverse))
            .andThen(target.reset.apply(Direction.kReverse).withTimeout(5)));
    autoChooser.addOption(
        target.name + "SysId Dynamic Forward",
        target
            .routine
            .dynamic(SysIdRoutine.Direction.kForward)
            .until(() -> target.safetyEscapeSupplier.apply(Direction.kForward))
            .andThen(target.reset.apply(Direction.kForward).withTimeout(5)));
    autoChooser.addOption(
        target.name + "SysId Dynamic Backward",
        target
            .routine
            .dynamic(SysIdRoutine.Direction.kReverse)
            .until(() -> target.safetyEscapeSupplier.apply(Direction.kReverse))
            .andThen(target.reset.apply(Direction.kReverse).withTimeout(5)));
    autoChooser.addOption(
        target.name + "All Sysid Routines(Recommended)",
        new SequentialCommandGroup(
            new PrintCommand("Starting " + target.name + " SysId Routines"),
            target.reset.apply(Direction.kForward).withTimeout(5),
            target
                .routine
                .quasistatic(SysIdRoutine.Direction.kForward)
                .until(() -> target.safetyEscapeSupplier.apply(Direction.kForward)),
            target.reset.apply(Direction.kForward).withTimeout(5),
            target
                .routine
                .quasistatic(Direction.kReverse)
                .until(() -> target.safetyEscapeSupplier.apply(Direction.kReverse)),
            target.reset.apply(Direction.kReverse).withTimeout(5),
            target
                .routine
                .dynamic(SysIdRoutine.Direction.kForward)
                .until(() -> target.safetyEscapeSupplier.apply(Direction.kForward)),
            target.reset.apply(Direction.kReverse).withTimeout(5),
            target
                .routine
                .dynamic(SysIdRoutine.Direction.kReverse)
                .until(() -> target.safetyEscapeSupplier.apply(Direction.kReverse)),
            target.reset.apply(Direction.kReverse).withTimeout(5),
            new PrintCommand("SysId Routines Complete")));
  }

  public static interface SysIdSingleSubsystem extends SysIdSubsystem {
    public SysIdRoutine getSysIdRoutine();

    public String getName();

    public Command reset(SysIdRoutine.Direction direction);

    public default List<SysIdTarget> getSysIdTargets() {
      return List.of(new SysIdTarget(getName(), getSysIdRoutine(), this::reset));
    }
  }
}
