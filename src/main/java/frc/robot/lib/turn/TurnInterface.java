/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib.turn;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.lib.pid.PIDOutput;
import frc.robot.lib.pid.PIDSource;

/**
 * Interface for subsystems to be turned using Turn
 */
public interface TurnInterface extends PIDSource, PIDOutput, Subsystem{
    public void resetAngle();

    public double getRate();

    public void stop();
}
