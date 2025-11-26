// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.utility.command;

import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

/**
 * A state machine representing a complete action to be performed by the robot. Commands are run by
 * the {@link CommandScheduler}, and can be composed into CommandGroups to allow users to build
 * complicated multistep actions without the need to roll the state machine logic themselves.
 *
 * <p>Commands are run synchronously from the main robot loop; no multithreading is used, unless
 * specified explicitly from the command implementation.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public abstract class Command{
  /** Requirements set. */
  private final Set<Subsystem> m_requirements = new HashSet<>();

  /** Default constructor. */
  protected Command() {}

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  public void initialize() {}

  /** The main body of a command. Called repeatedly while the command is scheduled. */
  public void execute() {}

  /**
   * The action to take when the command ends. Called when either the command finishes normally, or
   * when it interrupted/canceled.
   *
   * <p>Do not schedule commands here that share requirements with this command. Use {@link
   * #andThen(Command...)} instead.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  public void end(boolean interrupted) {}

  /**
   * Whether the command has finished. Once a command finishes, the scheduler will call its end()
   * method and un-schedule it.
   *
   * @return whether the command has finished.
   */
  public boolean isFinished() {
    return false;
  }

  /**
   * Specifies the set of subsystems used by this command. Two commands cannot use the same
   * subsystem at the same time. If another command is scheduled that shares a requirement, {@link
   * #getInterruptionBehavior()} will be checked and followed. If no subsystems are required, return
   * an empty set.
   *
   * <p>Note: it is recommended that user implementations contain the requirements as a field, and
   * return that field here, rather than allocating a new set every time this is called.
   *
   * @return the set of subsystems that are required
   * @see InterruptionBehavior
   */
  public Set<Subsystem> getRequirements() {
    return m_requirements;
  }

  /**
   * Adds the specified subsystems to the requirements of the command. The scheduler will prevent
   * two commands that require the same subsystem from being scheduled simultaneously.
   *
   * <p>Note that the scheduler determines the requirements of a command when it is scheduled, so
   * this method should normally be called from the command's constructor.
   *
   * @param requirements the requirements to add
   */
  public final void addRequirements(Subsystem... requirements) {
      m_requirements.addAll(Arrays.asList(requirements));
  }

  /**
   * Adds the specified subsystems to the requirements of the command. The scheduler will prevent
   * two commands that require the same subsystem from being scheduled simultaneously.
   *
   * <p>Note that the scheduler determines the requirements of a command when it is scheduled, so
   * this method should normally be called from the command's constructor.
   *
   * @param requirements the requirements to add
   */
  public final void addRequirements(Collection<Subsystem> requirements) {
    m_requirements.addAll(requirements);
  }

  /**
   * Decorates this command with a timeout. If the specified timeout is exceeded before the command
   * finishes normally, the command will be interrupted and un-scheduled.
   *
   * <p>Note: This decorator works by adding this command to a composition. The command the
   * decorator was called on cannot be scheduled independently or be added to a different
   * composition (namely, decorators), unless it is manually cleared from the list of composed
   * commands with {@link CommandScheduler#removeComposedCommand(Command)}. The command composition
   * returned from this method can be further decorated without issue.
   *
   * @param seconds the timeout duration
   * @return the command with the timeout added
   */
  public ParallelRaceGroup withTimeout(double seconds) {
    return raceWith(new WaitCommand(seconds));
  }

  /**
   * Decorates this command with an interrupt condition. If the specified condition becomes true
   * before the command finishes normally, the command will be interrupted and un-scheduled.
   *
   * <p>Note: This decorator works by adding this command to a composition. The command the
   * decorator was called on cannot be scheduled independently or be added to a different
   * composition (namely, decorators), unless it is manually cleared from the list of composed
   * commands with {@link CommandScheduler#removeComposedCommand(Command)}. The command composition
   * returned from this method can be further decorated without issue.
   *
   * @param condition the interrupt condition
   * @return the command with the interrupt condition added
   * @see #onlyWhile(BooleanSupplier)
   */
  public ParallelRaceGroup until(BooleanSupplier condition) {
    return raceWith(new WaitUntilCommand(condition));
  }

  /**
   * Decorates this command with a run condition. If the specified condition becomes false before
   * the command finishes normally, the command will be interrupted and un-scheduled.
   *
   * <p>Note: This decorator works by adding this command to a composition. The command the
   * decorator was called on cannot be scheduled independently or be added to a different
   * composition (namely, decorators), unless it is manually cleared from the list of composed
   * commands with {@link CommandScheduler#removeComposedCommand(Command)}. The command composition
   * returned from this method can be further decorated without issue.
   *
   * @param condition the run condition
   * @return the command with the run condition added
   * @see #until(BooleanSupplier)
   */
  public ParallelRaceGroup onlyWhile(BooleanSupplier condition) {
    return until(() -> !condition.getAsBoolean());
  }

  /**
   * Decorates this command with a runnable to run before this command starts.
   *
   * <p>Note: This decorator works by adding this command to a composition. The command the
   * decorator was called on cannot be scheduled independently or be added to a different
   * composition (namely, decorators), unless it is manually cleared from the list of composed
   * commands with {@link CommandScheduler#removeComposedCommand(Command)}. The command composition
   * returned from this method can be further decorated without issue.
   *
   * @param toRun the Runnable to run
   * @param requirements the required subsystems
   * @return the decorated command
   */
  public SequentialCommandGroup beforeStarting(Runnable toRun, Subsystem... requirements) {
    return beforeStarting(new InstantCommand(toRun, requirements));
  }

  /**
   * Decorates this command with another command to run before this command starts.
   *
   * <p>Note: This decorator works by adding this command to a composition. The command the
   * decorator was called on cannot be scheduled independently or be added to a different
   * composition (namely, decorators), unless it is manually cleared from the list of composed
   * commands with {@link CommandScheduler#removeComposedCommand(Command)}. The command composition
   * returned from this method can be further decorated without issue.
   *
   * @param before the command to run before this one
   * @return the decorated command
   */
  public SequentialCommandGroup beforeStarting(Command before) {
    return new SequentialCommandGroup(before, this);
  }

  /**
   * Decorates this command with a runnable to run after the command finishes.
   *
   * <p>Note: This decorator works by adding this command to a composition. The command the
   * decorator was called on cannot be scheduled independently or be added to a different
   * composition (namely, decorators), unless it is manually cleared from the list of composed
   * commands with {@link CommandScheduler#removeComposedCommand(Command)}. The command composition
   * returned from this method can be further decorated without issue.
   *
   * @param toRun the Runnable to run
   * @param requirements the required subsystems
   * @return the decorated command
   */
  public SequentialCommandGroup andThen(Runnable toRun, Subsystem... requirements) {
    return andThen(new InstantCommand(toRun, requirements));
  }

  /**
   * Decorates this command with a set of commands to run after it in sequence. Often more
   * convenient/less-verbose than constructing a new {@link SequentialCommandGroup} explicitly.
   *
   * <p>Note: This decorator works by adding this command to a composition. The command the
   * decorator was called on cannot be scheduled independently or be added to a different
   * composition (namely, decorators), unless it is manually cleared from the list of composed
   * commands with {@link CommandScheduler#removeComposedCommand(Command)}. The command composition
   * returned from this method can be further decorated without issue.
   *
   * @param next the commands to run next
   * @return the decorated command
   */
  public SequentialCommandGroup andThen(Command... next) {
    SequentialCommandGroup group = new SequentialCommandGroup(this);
    group.addCommands(next);
    return group;
  }

  /**
   * Creates a new command that runs this command and the deadline in parallel, finishing (and
   * interrupting this command) when the deadline finishes.
   *
   * <p>Note: This decorator works by adding this command to a composition. The command the
   * decorator was called on cannot be scheduled independently or be added to a different
   * composition (namely, decorators), unless it is manually cleared from the list of composed
   * commands with {@link CommandScheduler#removeComposedCommand(Command)}. The command composition
   * returned from this method can be further decorated without issue.
   *
   * @param deadline the deadline of the command group
   * @return the decorated command
   * @see Command#deadlineFor
   */
  public ParallelDeadlineGroup withDeadline(Command deadline) {
    return new ParallelDeadlineGroup(deadline, this);
  }

  /**
   * Decorates this command with a set of commands to run parallel to it, ending when the calling
   * command ends and interrupting all the others. Often more convenient/less-verbose than
   * constructing a new {@link ParallelDeadlineGroup} explicitly.
   *
   * <p>Note: This decorator works by adding this command to a composition. The command the
   * decorator was called on cannot be scheduled independently or be added to a different
   * composition (namely, decorators), unless it is manually cleared from the list of composed
   * commands with {@link CommandScheduler#removeComposedCommand(Command)}. The command composition
   * returned from this method can be further decorated without issue.
   *
   * @param parallel the commands to run in parallel. Note the parallel commands will be interrupted
   *     when the deadline command ends
   * @return the decorated command
   * @see Command#withDeadline
   */
  public ParallelDeadlineGroup deadlineFor(Command... parallel) {
    return new ParallelDeadlineGroup(this, parallel);
  }

  /**
   * Decorates this command with a set of commands to run parallel to it, ending when the last
   * command ends. Often more convenient/less-verbose than constructing a new {@link
   * ParallelCommandGroup} explicitly.
   *
   * <p>Note: This decorator works by adding this command to a composition. The command the
   * decorator was called on cannot be scheduled independently or be added to a different
   * composition (namely, decorators), unless it is manually cleared from the list of composed
   * commands with {@link CommandScheduler#removeComposedCommand(Command)}. The command composition
   * returned from this method can be further decorated without issue.
   *
   * @param parallel the commands to run in parallel
   * @return the decorated command
   */
  public ParallelCommandGroup alongWith(Command... parallel) {
    ParallelCommandGroup group = new ParallelCommandGroup(this);
    group.addCommands(parallel);
    return group;
  }

  /**
   * Decorates this command with a set of commands to run parallel to it, ending when the first
   * command ends. Often more convenient/less-verbose than constructing a new {@link
   * ParallelRaceGroup} explicitly.
   *
   * <p>Note: This decorator works by adding this command to a composition. The command the
   * decorator was called on cannot be scheduled independently or be added to a different
   * composition (namely, decorators), unless it is manually cleared from the list of composed
   * commands with {@link CommandScheduler#removeComposedCommand(Command)}. The command composition
   * returned from this method can be further decorated without issue.
   *
   * @param parallel the commands to run in parallel
   * @return the decorated command
   */
  public ParallelRaceGroup raceWith(Command... parallel) {
    ParallelRaceGroup group = new ParallelRaceGroup(this);
    group.addCommands(parallel);
    return group;
  }

  /**
   * Decorates this command to run repeatedly, restarting it when it ends, until this command is
   * interrupted. The decorated command can still be canceled.
   *
   * <p>Note: This decorator works by adding this command to a composition. The command the
   * decorator was called on cannot be scheduled independently or be added to a different
   * composition (namely, decorators), unless it is manually cleared from the list of composed
   * commands with {@link CommandScheduler#removeComposedCommand(Command)}. The command composition
   * returned from this method can be further decorated without issue.
   *
   * @return the decorated command
   */
  public RepeatCommand repeatedly() {
    return new RepeatCommand(this);
  }

  /**
   * Decorates this command to only run if this condition is not met. If the command is already
   * running and the condition changes to true, the command will not stop running. The requirements
   * of this command will be kept for the new conditional command.
   *
   * <p>Note: This decorator works by adding this command to a composition. The command the
   * decorator was called on cannot be scheduled independently or be added to a different
   * composition (namely, decorators), unless it is manually cleared from the list of composed
   * commands with {@link CommandScheduler#removeComposedCommand(Command)}. The command composition
   * returned from this method can be further decorated without issue.
   *
   * @param condition the condition that will prevent the command from running
   * @return the decorated command
   * @see #onlyIf(BooleanSupplier)
   */
  public ConditionalCommand unless(BooleanSupplier condition) {
    return new ConditionalCommand(new InstantCommand(), this, condition);
  }

  /**
   * Decorates this command to only run if this condition is met. If the command is already running
   * and the condition changes to false, the command will not stop running. The requirements of this
   * command will be kept for the new conditional command.
   *
   * <p>Note: This decorator works by adding this command to a composition. The command the
   * decorator was called on cannot be scheduled independently or be added to a different
   * composition (namely, decorators), unless it is manually cleared from the list of composed
   * commands with {@link CommandScheduler#removeComposedCommand(Command)}. The command composition
   * returned from this method can be further decorated without issue.
   *
   * @param condition the condition that will allow the command to run
   * @return the decorated command
   * @see #unless(BooleanSupplier)
   */
  public ConditionalCommand onlyIf(BooleanSupplier condition) {
    return unless(() -> !condition.getAsBoolean());
  }

  /**
   * Cancels this command. Will call {@link #end(boolean) end(true)}.
   *
   * @see CommandScheduler#cancel(Command...)
   */
  public void cancel() {
    CommandScheduler.getInstance().cancel(this);
  }

  /**
   * Whether the command is currently scheduled. Note that this does not detect whether the command
   * is in a composition, only whether it is directly being run by the scheduler.
   *
   * @return Whether the command is scheduled.
   */
  public boolean isScheduled() {
    return CommandScheduler.getInstance().isScheduled(this);
  }

  /**
   * Whether the command requires a given subsystem.
   *
   * @param requirement the subsystem to inquire about
   * @return whether the subsystem is required
   */
  public boolean hasRequirement(Subsystem requirement) {
    return getRequirements().contains(requirement);
  }
}
