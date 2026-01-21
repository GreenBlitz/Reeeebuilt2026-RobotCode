package frc.robot.statemachine.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.robot.Robot;
import frc.robot.statemachine.ShootingCalculations;
import frc.robot.statemachine.shooterstatehandler.ShooterConstants;
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

	private static boolean isTurretAtTarget(Rotation2d turretPosition, Rotation2d target, Rotation2d tolerance) {
		boolean isAtHeading = MathUtil.isNear(target.getDegrees(), turretPosition.getDegrees(), tolerance.getDegrees());
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
		Rotation2d flywheelVelocityToleranceRotation2dPerSecond,
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

		boolean isAtTurretAtTarget = isTurretAtTarget(
			robot.getTurret().getPosition(),
			ShootingCalculations.getShootingParams().targetTurretPosition(),
			headingTolerance
		);

		boolean isFlywheelReadyToShoot = isFlywheelAtVelocity(
			ShootingCalculations.getShootingParams().targetFlywheelVelocityRPS(),
			flywheelVelocityRotation2dPerSecond,
			flywheelVelocityToleranceRotation2dPerSecond
		);

		boolean isHoodAtPosition = isHoodAtPositon(
			ShootingCalculations.getShootingParams().targetHoodPosition(),
			hoodPosition,
			hoodPositionTolerance
		);

		return isFlywheelReadyToShoot && isHoodAtPosition && isInRange && isWithinDistance && isAtTurretAtTarget;
	}

	public static boolean calibrationIsReadyToShoot(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRotation2dPerSecond,
		Rotation2d hoodPositionTolerance
	) {
		Rotation2d flywheelVelocityRotation2dPerSecond = robot.getFlyWheel().getVelocity();
		Rotation2d hoodPosition = robot.getHood().getPosition();

		boolean isFlywheelReadyToShoot = isFlywheelAtVelocity(
			ShooterConstants.flywheelCalibrationRotations.get(),
			flywheelVelocityRotation2dPerSecond,
			flywheelVelocityToleranceRotation2dPerSecond
		);

		boolean isHoodAtPosition = isHoodAtPositon(ShooterConstants.hoodCalibrationAngle.get(), hoodPosition, hoodPositionTolerance);

		return isFlywheelReadyToShoot && isHoodAtPosition;
	}

}
