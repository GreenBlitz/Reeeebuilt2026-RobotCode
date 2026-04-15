package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.field.Field;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.utils.math.FieldMath;

public class TowerAssistCalculations {

	public static boolean isInFrontOfClosestTower(Pose2d robotPose) {
		Translation2d blueTower = Field.TOWER_MIDDLE;
		Translation2d redTower = FieldMath.mirror(Field.TOWER_MIDDLE, true, true);
		Translation2d closestTower = robotPose.getX() < Field.LENGTH_METERS / 2 ? blueTower : redTower;

		return Math.abs(robotPose.getY() - closestTower.getY())
			< Field.TOWER_Y_AXIS_LENGTH_METERS / 2 + RobotConstants.DISTANCE_FROM_ROBOT_CENTER_TO_HOPPER_EDGE_WHEN_OPENED_METERS;
	}

	public static boolean isInNeutralZone(Pose2d robotPose) {
		return robotPose.getX() > Field.ALLIANCE_START_LINE_X_VALUE && robotPose.getX() < FieldMath.mirrorX(Field.ALLIANCE_START_LINE_X_VALUE);
	}

	public static Pose2d getTowerAssistTarget(Pose2d robotPose) {
		Translation2d blueTower = Field.TOWER_MIDDLE;
		Translation2d redTower = FieldMath.mirror(Field.TOWER_MIDDLE, true, true);
		boolean isNearBlueTower = robotPose.getX() < Field.LENGTH_METERS / 2;
		Translation2d closestTower = isNearBlueTower ? blueTower : redTower;

		boolean isOnOutpostSide = robotPose.getY() < closestTower.getY();
		double xOffset = isNearBlueTower ? -Field.TOWER_MIDDLE.getX() / 2 : Field.TOWER_MIDDLE.getX() / 2;
		double yOffset = isOnOutpostSide ? -Field.TOWER_ASSIST_Y_OFFSET : Field.TOWER_ASSIST_Y_OFFSET;

		Rotation2d targetRotation = isOnOutpostSide ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg;

		return new Pose2d(closestTower.plus(new Translation2d(xOffset, yOffset)), targetRotation);
	}

	public static ChassisSpeeds getTowerAssistedSpeeds(ChassisPowers driverInputs, Robot robot, Pose2d assistTarget, SwerveState swerveState) {
		ChassisSpeeds driverSpeeds = SwerveMath.powersToSpeeds(driverInputs, robot.getSwerve().getConstants());
		Pose2d currentPose = robot.getPoseEstimator().getEstimatedPose();

		return AimAssistMath.getRotationAssistedSpeeds(
			AimAssistMath.getObjectAssistedSpeeds(
				driverSpeeds,
				currentPose,
				Rotation2d.kCW_90deg,
				assistTarget.getTranslation(),
				robot.getSwerve().getConstants(),
				swerveState
			),
			currentPose.getRotation(),
			assistTarget.getRotation(),
			robot.getSwerve().getConstants()
		);
	}

}
