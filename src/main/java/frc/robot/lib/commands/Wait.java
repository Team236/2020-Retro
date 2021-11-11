package frc.robot.lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *
 */
public class Wait extends CommandBase {
	double waitTime;
	int i = 0;
	double dt = (1 / 50.0);

	public Wait(double seconds) {
		this.waitTime = seconds;
	}
	
	public Wait(double seconds, double _dt) {
		this(seconds);
		this.dt = _dt;
	}

	@Override
	public void initialize() {
		i = 0;
	}

	@Override
	public void execute() {
		i++;
	}

	@Override
	public boolean isFinished() {
		return (i / 50.0) > waitTime;
	}

	@Override
	public void end(boolean interrupted) {
	}
}
