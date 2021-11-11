/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib.turn;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.pid.PID;
import frc.robot.lib.pid.PIDParameters;

public class Turn extends CommandBase {

  private PID pid;
  private double degrees;
  private double margin;
  private double angleError;

  private TurnInterface turnSubsystem;

  /**
   * Uses PID to turn a given subsystem a given number of degrees
   * @param _turnSubsystem The subsystem to be turned (must implement TurnInterface)
   * @param _degrees The angle of turn
   * @param _margin The margin of error of the turn
   * @param _pidParams PID parameters for the turn
   */
  public Turn(TurnInterface _turnSubsystem, double _degrees, double _margin, PIDParameters _pidParams) {
    this.turnSubsystem = _turnSubsystem;
    addRequirements(turnSubsystem);

    this.degrees = _degrees;
    this.margin = _margin;
    
    pid = new PID(this.turnSubsystem, this.turnSubsystem, _pidParams);
  }

  @Override
  public void initialize() {
    pid.setSetpoint(degrees);

    turnSubsystem.resetAngle();

    pid.enable();
    pid.update();
  }

  @Override
  public void execute() {
    angleError = pid.getError();
    SmartDashboard.putNumber("Angle errror", angleError);
  }

  @Override
  public void end(boolean interrupted) {
    pid.disable();
    turnSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    // TODO input target rate
    return ((Math.abs(angleError) < margin) && turnSubsystem.getRate() < 0.25);
  }
}
