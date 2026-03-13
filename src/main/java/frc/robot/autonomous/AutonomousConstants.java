package frc.robot.autonomous;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutonomousConstants {

	public static final String LOG_PATH_PREFIX = "Autonomous";

	public static final double PATHFINDING_DEADBAND_METERS = 0.5;

	public static final PathConstraints DEFAULT_PATHFINDING_CONSTRAINTS = new PathConstraints(
		4.200,
		2.000,
		Rotation2d.fromDegrees(540).getRadians(),
		Rotation2d.fromDegrees(720).getRadians()
	);

	public static final Pose2d DEFAULT_IS_NEAR_END_OF_PATH_TOLERANCE = new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(1));

	public static final double TIME_TO_WAIT_TO_CLOSE_INTAKE_AFTER_PATH_END_SECONDS = 4.0;
	public static final double TIME_TO_WAIT_TO_START_SHOOTING_AFTER_AUTO_START = 2.0;
	public static final double TIME_BETWEEN_WIGGLES_AFTER_PATH_END_SECONDS = 0.2;
	public static final Rotation2d WIGGLE_ANGLE_AFTER_PATH_END_SECONDS = Rotation2d.fromDegrees(7);


}
