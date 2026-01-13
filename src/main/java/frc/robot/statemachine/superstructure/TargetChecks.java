package frc.robot.statemachine.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.robot.Robot;
import frc.robot.statemachine.ShooterCalculations;
import frc.robot.subsystems.arm.Arm;
import frc.utils.math.FieldMath;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.MathUtil.isNear;

public class TargetChecks {

	private final Superstructure superstructure;
	private static final String isReadyToShootLogPath = "Statemachine/TargetChecks/IsReadyToShoot";

	public TargetChecks(Superstructure superstructure) {
		this.superstructure = superstructure;
	}

	private static boolean isWithinDistance(Translation2d robotPosition, double maxShootingDistanceFromTargetMeters) {
		boolean isWithinDistance = robotPosition.getDistance(Field.getHubMiddle()) <= maxShootingDistanceFromTargetMeters;
		Logger.recordOutput(isReadyToShootLogPath + "/isInDistance", isWithinDistance);
		return isWithinDistance;
	}

	private static boolean isInAngleRange(Translation2d robotPosition, Rotation2d maxAngleFromCenter) {
		Rotation2d AngleBetweenRobotAndGoal = FieldMath.getRelativeTranslation(Field.getHubMiddle(), robotPosition).getAngle();
		boolean isInAngleRange = Math.abs(AngleBetweenRobotAndGoal.getDegrees()) <= maxAngleFromCenter.getDegrees();
		Logger.recordOutput(isReadyToShootLogPath + "/isInRange", isInAngleRange);
		return isInAngleRange;
	}

	private static boolean isTurretAtTarget(Pose2d robotPose, Arm turret, double tolerance) {
		Rotation2d wantedAngle = ShooterCalculations.getRobotRelativeLookAtHubAngleForTurret(robotPose, turret.getPosition());
		boolean isAtHeading = MathUtil.isNear(wantedAngle.getDegrees(), turret.getPosition().getDegrees(), tolerance);
		Logger.recordOutput(isReadyToShootLogPath + "/isAtHeading", isAtHeading);
		return isAtHeading;
	}

	private static boolean isFlywheelAtVelocity(
		Rotation2d wantedFlywheelVelocityRotation2dPerSecond,
		Rotation2d flywheelVelocityRotation2dPerSecond,
		Rotation2d flywheelVelocityToleranceRotation2dPerSecond
	) {
		boolean isFlywheelAtVelocity = isNear(
			wantedFlywheelVelocityRotation2dPerSecond.getDegrees(),
			flywheelVelocityRotation2dPerSecond.getDegrees(),
			flywheelVelocityToleranceRotation2dPerSecond.getDegrees()
		);
		Logger.recordOutput(isReadyToShootLogPath + "/isFlywheelAtVelocity", isFlywheelAtVelocity);
		return isFlywheelAtVelocity;
	}

	private static boolean isHoodAtPositon(Rotation2d wantedPosition, Rotation2d hoodPosition, Rotation2d tolerance) {
		boolean isHoodAtPosition = MathUtil.isNear(wantedPosition.getDegrees(), hoodPosition.getDegrees(), tolerance.getDegrees());
		Logger.recordOutput(isReadyToShootLogPath + "/isHoodAtPositon", isHoodAtPosition);
		return isHoodAtPosition;
	}

	public static boolean isReadyToShoot(
		Robot robot,
		Rotation2d wantedFlywheelVelocityRPS,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d wantedHoodPosition,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromHubCenter,
		double maxShootingDistanceFromTargetMeters
	) {
		Pose2d robotPose = robot.getPoseEstimator().getEstimatedPose();
		Rotation2d flywheelVelocityRotation2dPerSecond = robot.getFlyWheel().getVelocity();
		Rotation2d hoodPosition = robot.getHood().getPosition();

		boolean isWithinDistance = isWithinDistance(robotPose.getTranslation(), maxShootingDistanceFromTargetMeters);

		boolean isInRange = isInAngleRange(robotPose.getTranslation(), maxAngleFromHubCenter);

		boolean isAtHeading = isTurretAtTarget(robotPose, robot.getTurret(), headingTolerance.getDegrees());

		boolean isFlywheelReadyToShoot = isFlywheelAtVelocity(
			wantedFlywheelVelocityRPS,
			flywheelVelocityRotation2dPerSecond,
			flywheelVelocityToleranceRPS
		);

		boolean isHoodAtPosition = isHoodAtPositon(wantedHoodPosition, hoodPosition, hoodPositionTolerance);

		return isFlywheelReadyToShoot && isHoodAtPosition && isInRange && isWithinDistance && isAtHeading;
	}

}
