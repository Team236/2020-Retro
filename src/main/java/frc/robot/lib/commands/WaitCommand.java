package frc.robot.lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// import edu.wpi.first.wpilibj.command.Command;
// import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class WaitCommand extends SequentialCommandGroup {

	/**
	 * Start a command on a delay
	 * @param delay The delay before the command starts, in seconds
	 * @param command The command to start
	 */
	public WaitCommand(double delay, Command command) {
		addCommands(new Wait(delay), 
			command);
	}

	/**
	 * Start a command on a delay that times out
	 * @param delay The delay before command starts, in seconds
	 * @param command The command to start
	 * @param timeout The amount of time the command should run for, in seconds
	 */
	public WaitCommand(double delay, Command command, double timeout) {
		addCommands(new Wait(delay),
			new TimeoutCommand(command, timeout));
	}

}
