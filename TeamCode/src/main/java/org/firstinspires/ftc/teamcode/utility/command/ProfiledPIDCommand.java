// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.utility.command;


import org.firstinspires.ftc.teamcode.utility.math.controller.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.utility.math.trajectory.TrapezoidProfile;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * A command that controls an output with a {@link ProfiledPIDController}. Runs forever by default -
 * to add exit conditions and/or other behavior, subclass this class. The controller calculation and
 * output are performed synchronously in the command's execute() method.
 *
 * <p>This class is provided by the NewCommands VendorDep
 *
 * @deprecated Use a ProfiledPIDController instead
 */
public class ProfiledPIDCommand extends Command {
  /** Profiled PID controller. */
  protected final ProfiledPIDController m_controller;

  /** Measurement getter. */
  protected DoubleSupplier m_measurement;

  /** Goal getter. */
  protected Supplier<TrapezoidProfile.State> m_goal;

  /** Profiled PID controller output consumer. */
  protected BiConsumer<Double, TrapezoidProfile.State> m_useOutput;

  /**
   * Creates a new PIDCommand, which controls the given output with a ProfiledPIDController. Goal
   * velocity is specified.
   *
   * @param controller the controller that controls the output.
   * @param measurementSource the measurement of the process variable
   * @param goalSource the controller's goal
   * @param useOutput the controller's output
   * @param requirements the subsystems required by this command
   */
  @SuppressWarnings("this-escape")
  public ProfiledPIDCommand(
      ProfiledPIDController controller,
      DoubleSupplier measurementSource,
      Supplier<TrapezoidProfile.State> goalSource,
      BiConsumer<Double, TrapezoidProfile.State> useOutput,
      Subsystem... requirements) {

    m_controller = controller;
    m_useOutput = useOutput;
    m_measurement = measurementSource;
    m_goal = goalSource;
    addRequirements(requirements);
  }

  /**
   * Creates a new PIDCommand, which controls the given output with a ProfiledPIDController. Goal
   * velocity is implicitly zero.
   *
   * @param controller the controller that controls the output.
   * @param measurementSource the measurement of the process variable
   * @param goalSource the controller's goal
   * @param useOutput the controller's output
   * @param requirements the subsystems required by this command
   */
  @SuppressWarnings("this-escape")
  public ProfiledPIDCommand(
      ProfiledPIDController controller,
      DoubleSupplier measurementSource,
      DoubleSupplier goalSource,
      BiConsumer<Double, TrapezoidProfile.State> useOutput,
      Subsystem... requirements) {

    m_controller = controller;
    m_useOutput = useOutput;
    m_measurement = measurementSource;
    m_goal = () -> new TrapezoidProfile.State(goalSource.getAsDouble(), 0);
    addRequirements(requirements);
  }

  /**
   * Creates a new PIDCommand, which controls the given output with a ProfiledPIDController. Goal
   * velocity is specified.
   *
   * @param controller the controller that controls the output.
   * @param measurementSource the measurement of the process variable
   * @param goal the controller's goal
   * @param useOutput the controller's output
   * @param requirements the subsystems required by this command
   */
  public ProfiledPIDCommand(
      ProfiledPIDController controller,
      DoubleSupplier measurementSource,
      TrapezoidProfile.State goal,
      BiConsumer<Double, TrapezoidProfile.State> useOutput,
      Subsystem... requirements) {
    this(controller, measurementSource, () -> goal, useOutput, requirements);
  }

  /**
   * Creates a new PIDCommand, which controls the given output with a ProfiledPIDController. Goal
   * velocity is implicitly zero.
   *
   * @param controller the controller that controls the output.
   * @param measurementSource the measurement of the process variable
   * @param goal the controller's goal
   * @param useOutput the controller's output
   * @param requirements the subsystems required by this command
   */
  public ProfiledPIDCommand(
      ProfiledPIDController controller,
      DoubleSupplier measurementSource,
      double goal,
      BiConsumer<Double, TrapezoidProfile.State> useOutput,
      Subsystem... requirements) {
    this(controller, measurementSource, () -> goal, useOutput, requirements);
  }

  @Override
  public void initialize() {
    m_controller.reset(m_measurement.getAsDouble());
  }

  @Override
  public void execute() {
    m_useOutput.accept(
        m_controller.calculate(m_measurement.getAsDouble(), m_goal.get()),
        m_controller.getSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    m_useOutput.accept(0.0, new TrapezoidProfile.State());
  }

  /**
   * Returns the ProfiledPIDController used by the command.
   *
   * @return The ProfiledPIDController
   */
  public ProfiledPIDController getController() {
    return m_controller;
  }
}
