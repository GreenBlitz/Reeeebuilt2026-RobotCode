package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.SimulationManager;

public class ShooterCalculations {

	public static Pose2d getTurretPose(Pose2d robotPose) {
		return new Pose2d(
			robotPose.getX() + robotPose.getRotation().getCos() * SimulationManager.TURRET_DISTANCE_FROM_ROBOT_ON_X_AXIS,
			robotPose.getY() + robotPose.getRotation().getSin() * SimulationManager.TURRET_DISTANCE_FROM_ROBOT_ON_X_AXIS,
			robotPose.getRotation()
		);
	}

}