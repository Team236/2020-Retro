package frc.robot.lib.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;


/**
 *
 */
public class Test extends CommandBase {
	private String name;

	public Test(String _name) {
		this.name = _name;
	}

	@Override
	public void initialize() {
		System.out.println("Test (init): " + name);
	}

	@Override
	public void execute() {
		System.out.println("Test (exec): " + name);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			System.out.println("Test (interrupt): " + name);

		} else {
			System.out.println("Test (end): " + name);

		}
	}
}
