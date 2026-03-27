package frc.utils;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.IPs;
import frc.robot.Robot;
import frc.utils.time.TimeUtil;

import java.nio.file.Path;

public class OperatorTimerClicker extends Command {

	private static final String CLICK_POINT_TOPIC_NAME = "ClickPoint/ShouldClick";

	private final BooleanEntry shouldClickEntry;
	private double startTime;

	public OperatorTimerClicker() {
		this.shouldClickEntry = NetworkTableInstance.getDefault().getBooleanTopic(CLICK_POINT_TOPIC_NAME).getEntry(false);
		shouldClickEntry.set(false);

		if (Robot.ROBOT_TYPE.isSimulation()) {
			CMDHandler.runPythonClass(Path.of("PointClicker"), IPs.SIMULATION_IP);
		}
	}

	@Override
	public void initialize() {
		shouldClickEntry.set(true);
		this.startTime = TimeUtil.getCurrentTimeSeconds();
	}

	@Override
	public void end(boolean interrupted) {
		shouldClickEntry.set(false);
	}

	@Override
	public boolean isFinished() {
		return TimeUtil.getCurrentTimeSeconds() > (startTime + 0.5);
	}

}
