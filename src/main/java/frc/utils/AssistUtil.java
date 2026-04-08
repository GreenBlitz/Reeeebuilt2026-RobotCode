package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.utils.math.FieldMath;

public class AssistUtil {

	public static final double DEPOT_Y_OFFSET_FOR_TOWER = 1.2;
	private static final double OUTPOST_Y_OFFSET_FOR_TOWER = -1.2;

	public static boolean isRobotOnOutpostSide(Pose2d currentPose) {
		return currentPose.getY() < Field.TOWER_MIDDLE.getY();
	}

	public static boolean shouldMirror(Pose2d currentPose) {
		return currentPose.getX() > Field.LENGTH_METERS / 2;
	}

	public static double towerYOffest(Pose2d currentPose) {
		double yOffset = isRobotOnOutpostSide(currentPose) ? DEPOT_Y_OFFSET_FOR_TOWER : OUTPOST_Y_OFFSET_FOR_TOWER;
		if (shouldMirror(currentPose)) {
			yOffset += -1;
		}
		return yOffset;
	}

	public static Translation2d towerOffset(Pose2d currentPose) {
		return new Translation2d(Field.TOWER_MIDDLE.getX() / 2, towerYOffest(currentPose));
	}

	public static Translation2d targetTowerTranslation(Pose2d currentPose) {
		Translation2d targetTranslation = Field.TOWER_MIDDLE.minus(towerOffset(currentPose));
		targetTranslation = FieldMath.mirror(targetTranslation, shouldMirror(currentPose), shouldMirror(currentPose));
		return targetTranslation;
	}

	public static Rotation2d targetRotation(Pose2d currentPose) {
		return (isRobotOnOutpostSide(currentPose) ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg);
	}

	public static Pose2d finalTargetPose(Pose2d currentPose) {
		return new Pose2d(targetTowerTranslation(currentPose), targetRotation(currentPose));
	}

}
