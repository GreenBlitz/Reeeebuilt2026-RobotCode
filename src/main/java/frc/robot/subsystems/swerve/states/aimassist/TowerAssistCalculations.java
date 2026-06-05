package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.robot.RobotConstants;
import frc.utils.math.FieldMath;

public class TowerAssistCalculations {

	public static Translation2d getClosestTower(Pose2d robotPose) {
		Translation2d blueTower = Field.TOWER_MIDDLE;
		Translation2d redTower = FieldMath.mirror(Field.TOWER_MIDDLE, true, true);
		boolean isNearBlueTower = robotPose.getX() < Field.LENGTH_METERS / 2;
		return isNearBlueTower ? blueTower : redTower;
	}

	public static Pose2d getAssistTarget(Pose2d robotPose) {
		Translation2d closestTower = getClosestTower(robotPose);
		boolean isOnOutpostSide = robotPose.getY() < closestTower.getY();
		Rotation2d targetRotation = isOnOutpostSide ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg;

		return new Pose2d(closestTower, targetRotation);
	}

	public static boolean isInFrontOfClosestTower(Pose2d robotPose) {
		return Math.abs(robotPose.getY() - getClosestTower(robotPose).getY())
			< Field.TOWER_Y_AXIS_LENGTH_METERS / 2 + RobotConstants.DISTANCE_FROM_ROBOT_CENTER_TO_HOPPER_EDGE_WHEN_OPENED_METERS;
	}

	public static boolean isInNeutralZone(Pose2d robotPose) {
		return robotPose.getX() > Field.ALLIANCE_START_LINE_X_VALUE && robotPose.getX() < FieldMath.mirrorX(Field.ALLIANCE_START_LINE_X_VALUE);
	}

}
