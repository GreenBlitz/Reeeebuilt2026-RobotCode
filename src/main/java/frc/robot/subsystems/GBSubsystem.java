package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import org.littletonrobotics.junction.Logger;


public abstract class GBSubsystem extends SubsystemBase {

	private final String logPath;
	private Command currentCommand;
	private boolean isSubsystemRunningIndependently;

	public GBSubsystem(String logPath) {
		this.logPath = logPath;
		this.currentCommand = Commands.none().withName("None");
	}

	@Override
	public Command getCurrentCommand() {
		return currentCommand;
	}

	public String getLogPath() {
		return logPath;
	}

	public boolean isSubsystemRunningIndependently() {
		return isSubsystemRunningIndependently;
	}

	public void setIsSubsystemRunningIndependently(boolean isSubsystemRunningIndependently) {
		this.isSubsystemRunningIndependently = isSubsystemRunningIndependently;
	}

	@Override
	public final void periodic() {
		Logger.recordOutput(getLogPath() + "/CurrentCommand", getCurrentCommand().getName());
		subsystemPeriodic();
	}

	protected void subsystemPeriodic() {}

	public Command asSubsystemCommand(Command command, String commandName) {
		command.setName(commandName);
		command.addRequirements(this);

		return command.beforeStarting(new InstantCommand(() -> {
			currentCommand = command;
			setIsSubsystemRunningIndependently(true);
		})).andThen(new InstantCommand(() -> setIsSubsystemRunningIndependently(false)));
	}

}
