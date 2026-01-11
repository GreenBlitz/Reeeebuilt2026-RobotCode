package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.utils.math.FieldMath;
import org.littletonrobotics.junction.Logger;

public class PositionTargets {

	private final Robot robot;
    private static final String isInPoseToShootLogPath = "Statemachine/TargetChecks/IsInPoseToShoot";

    public PositionTargets(Robot robot) {
		this.robot = robot;
	}

    public static boolean isWithinDistance(Translation2d robotPosition, double maxShootingDistanceFromTargetMeters, Translation2d closestGoal) {
        boolean isWithinDistance = robotPosition.getDistance(closestGoal) <= maxShootingDistanceFromTargetMeters;
        Logger.recordOutput(isInPoseToShootLogPath + "/isInDistance", isWithinDistance);
        return isWithinDistance;
    }

    public static boolean isInAngleRange(Translation2d robotPosition, Pose2d closestGoal, Rotation2d maxAngleFromCenter) {
        Rotation2d AngleBetweenRobotAndGoal = FieldMath.getRelativeTranslation(closestGoal, robotPosition).getAngle();
        boolean isInAngleRange = Math.abs(AngleBetweenRobotAndGoal.getDegrees()) <= maxAngleFromCenter.getDegrees();
        Logger.recordOutput(isInPoseToShootLogPath + "/isInRange", isInAngleRange);
        return isInAngleRange;
    }
}
