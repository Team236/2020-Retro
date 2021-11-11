/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

public class TimeoutCommand extends ParallelRaceGroup {
  
  /**
   * Runs a command for specified amount of time
   * @param command Command to run
   * @param timeout Desired length of Command runtime, in seconds
   */
  public TimeoutCommand(Command command, double timeout) {

    super(command, 
      new Wait(timeout));
  }
}
