package frc.robot.subsystems.swerve.states.aimassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.robot.RobotConstants;
import frc.utils.math.FieldMath;

public class TowerAssistCalculations {

	private static final double ROBOT_CLOSE_TO_DRIVER_STATION_WALL_THRESHOLD_METERS = 1.2;
	private static final double INTAKE_FACING_DRIVER_STATION_POINT_PRODUCT_THRESHOLD = -0.1;

	public static Translation2d getClosestTower(Pose2d robotPose) {
		Translation2d blueTower = Field.TOWER_MIDDLE;
		Translation2d redTower = FieldMath.mirror(Field.TOWER_MIDDLE, true, true);
		boolean isNearBlueTower = robotPose.getX() < Field.LENGTH_METERS / 2;
		return isNearBlueTower ? blueTower : redTower;
	}

	public static Rotation2d getRobotTargetRotation(Translation2d closestTower, Pose2d robotPose) {
		boolean isOnOutpostSide = robotPose.getY() < closestTower.getY();
		return isOnOutpostSide ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg;
	}

	public static boolean isInFrontOfClosestTower(Pose2d robotPose) {
		return Math.abs(robotPose.getY() - getClosestTower(robotPose).getY())
			< Field.TOWER_Y_AXIS_LENGTH_METERS / 2 + RobotConstants.DISTANCE_FROM_ROBOT_CENTER_TO_HOPPER_EDGE_WHEN_OPENED_METERS;
	}

	public static boolean isInNeutralZone(Pose2d robotPose) {
		return robotPose.getX() > Field.ALLIANCE_START_LINE_X_VALUE && robotPose.getX() < FieldMath.mirrorX(Field.ALLIANCE_START_LINE_X_VALUE);
	}

	public static boolean shouldTakeLongTurnToAvoidWall(Pose2d robotPose) {
		Translation2d closestTower = getClosestTower(robotPose);

		boolean isBlueSideTower = closestTower.getX() < Field.LENGTH_METERS / 2;

		double distanceFromDriverStationWall = isBlueSideTower ? robotPose.getX() : Field.LENGTH_METERS - robotPose.getX();

		boolean isCloseToDriverStationWall = distanceFromDriverStationWall < ROBOT_CLOSE_TO_DRIVER_STATION_WALL_THRESHOLD_METERS;

		Translation2d driverStationDirection = isBlueSideTower ? new Translation2d(1, 0) : new Translation2d(-1, 0);

		Translation2d intakeDirection = new Translation2d(-robotPose.getRotation().getCos(), -robotPose.getRotation().getSin());

		boolean doesIntakeFaceDriverStation = (intakeDirection.getX() * driverStationDirection.getX()
			+ intakeDirection.getY() * driverStationDirection.getY()
			< INTAKE_FACING_DRIVER_STATION_POINT_PRODUCT_THRESHOLD);
		boolean doesIntakeFaceAwayFromTower = robotPose.getY() > Field.WIDTH_METERS / 2
			? intakeDirection.getY() > 0
			: intakeDirection.getY() < 0;

		return isCloseToDriverStationWall && doesIntakeFaceDriverStation && doesIntakeFaceAwayFromTower;
	}

}
