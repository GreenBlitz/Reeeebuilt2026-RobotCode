package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.robot.Robot;
import frc.utils.math.FieldMath;
import org.littletonrobotics.junction.Logger;

public class PositionTargets {

	private final Robot robot;
	private static final String isInPoseToShootLogPath = "Statemachine/PositionTargets/IsInPoseToShoot";

	public PositionTargets(Robot robot) {
		this.robot = robot;
	}

	public static boolean isInAngleRange(Translation2d robotPosition, Rotation2d maxAngleFromHubCenter) {
		Rotation2d AngleBetweenRobotAndHub = FieldMath.getRelativeTranslation(Field.getHubMiddle(), robotPosition).getAngle();
		boolean isInAngleRange = Math.abs(AngleBetweenRobotAndHub.getDegrees()) <= maxAngleFromHubCenter.getDegrees();
		Logger.recordOutput(isInPoseToShootLogPath + "/isInRange", isInAngleRange);
		return isInAngleRange;
	}

	public static boolean isWithinDistance(Translation2d robotPosition, double maxShootingDistanceFromHubMeters) {
		boolean isWithinDistance = robotPosition.getDistance(Field.getHubMiddle()) <= maxShootingDistanceFromHubMeters;
		Logger.recordOutput(isInPoseToShootLogPath + "/isInDistance", isWithinDistance);
		return isWithinDistance;
	}

}
