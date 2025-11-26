// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.utility.command;

/**
 * A base for subsystems that handles registration in the constructor, and provides a more intuitive
 * method for setting the default command.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public abstract class SubsystemBase implements Subsystem {
  /** Constructor. Telemetry/log name defaults to the classname. */
  @SuppressWarnings("this-escape")
  public SubsystemBase() {
    String name = this.getClass().getSimpleName();
    name = name.substring(name.lastIndexOf('.') + 1);
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  /**
   * Constructor.
   *
   * @param name Name of the subsystem for telemetry and logging.
   */
  @SuppressWarnings("this-escape")
  public SubsystemBase(String name) {
    CommandScheduler.getInstance().registerSubsystem(this);
  }
}
