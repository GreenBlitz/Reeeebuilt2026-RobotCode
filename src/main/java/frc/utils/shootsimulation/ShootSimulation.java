package frc.utils.shootsimulation;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class ShootSimulation extends Command {

	private static final double GRAVITY = -9.80665;

	private final String logPath;
	private final Translation3d startPosFieldRelative;
	private final Translation3d startVelFieldRelative;
	private final double stopHeightMeters;
	private final Timer timer = new Timer();

	private Translation3d currentPos;

	public ShootSimulation(String logPath, Translation3d startPosFieldRelative, Translation3d startVelFieldRelative, double stopHeightMeters) {
		this.logPath = logPath;
		this.startPosFieldRelative = startPosFieldRelative;
		this.startVelFieldRelative = startVelFieldRelative;
		this.stopHeightMeters = stopHeightMeters;

		this.currentPos = startPosFieldRelative;
	}

	@Override
	public void initialize() {
		timer.restart();
	}

	@Override
	public void execute() {
		double currentTime = timer.get();

		double x = startPosFieldRelative.getX() + (startVelFieldRelative.getX() * currentTime);
		double y = startPosFieldRelative.getY() + (startVelFieldRelative.getY() * currentTime);
		double z = startPosFieldRelative.getZ() + (startVelFieldRelative.getZ() * currentTime) + (0.5 * GRAVITY * Math.pow(currentTime, 2));

		currentPos = new Translation3d(x, y, z);
		Logger.recordOutput("ShootSim/" + logPath, currentPos);
	}

	@Override
	public boolean isFinished() {
		double currentTime = timer.get();
		double currentVz = startVelFieldRelative.getZ() + (GRAVITY * currentTime);

		return currentPos.getZ() <= stopHeightMeters && currentVz < 0;
	}

}
