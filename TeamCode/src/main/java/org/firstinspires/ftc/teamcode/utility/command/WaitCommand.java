// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.utility.command;


import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A command that does nothing but takes a specified amount of time to finish.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class WaitCommand extends Command {
  /** The timer used for waiting. */
  protected ElapsedTime m_timer = new ElapsedTime();

  private final double m_duration;

  /**
   * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
   *
   * @param seconds the time to wait, in seconds
   */
  @SuppressWarnings("this-escape")
  public WaitCommand(double seconds) {
    m_duration = seconds;
  }

  @Override
  public void initialize() {
    m_timer.reset();
  }

  @Override
  public boolean isFinished() {
    return m_timer.seconds() > m_duration;
  }
}
