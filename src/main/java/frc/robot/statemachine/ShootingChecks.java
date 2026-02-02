package frc.robot.statemachine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.AllianceSide;
import frc.constants.field.Field;
import frc.robot.Robot;
import frc.robot.statemachine.shooterstatehandler.ShooterConstants;
import frc.utils.math.FieldMath;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.MathUtil.isNear;

public class ShootingChecks {

	private static final String shootingChacksLogPath = "Statemachine/ShootingChecks";

	public static boolean isInAllianceZone(Translation2d position) {
		if (Field.isFieldConventionAlliance()) {
			return position.getX() < Field.getHubMiddle().getX();
		}
		return position.getX() > Field.getHubMiddle().getX();
	}

	public static boolean isInPositionForPassing(Pose2d robotPose, String logPath) {
		Translation2d turretPosition = ShootingCalculations.getFieldRelativeTurretPosition(robotPose);
		boolean isInPositionForPassing = !(turretPosition.getY() < ShooterConstants.MAX_Y_FOR_UNPASSABLE_AREA
			&& turretPosition.getY() > ShooterConstants.MIN_Y_FOR_UNPASSABLE_AREA);
		Logger.recordOutput(logPath + "/isInPassingNoNoZone", isInPositionForPassing);
		return isInPositionForPassing;
	}

	public static boolean isOnBumpOrUnderTrench(Pose2d robotPose, String logPath) {
		Translation2d turretPosition = ShootingCalculations.getFieldRelativeTurretPosition(robotPose);
		Translation2d blueAllianceRelativeTurretPosition = Field.getAllianceRelative(turretPosition);
		boolean isOnBumpOrUnderTrench = (blueAllianceRelativeTurretPosition.getX()
			< Field.getAllianceRelative(Field.getTrenchMiddle(AllianceSide.DEPOT)).getX() + (Field.TRENCH_X_AXIS_LENGTH_METERS / 2)
			&& blueAllianceRelativeTurretPosition.getX()
				> Field.getAllianceRelative(Field.getTrenchMiddle(AllianceSide.DEPOT)).getX() - (Field.TRENCH_X_AXIS_LENGTH_METERS / 2));
		Logger.recordOutput(logPath + "/isOnBumpOrUnderTranch", isOnBumpOrUnderTrench);
		return isOnBumpOrUnderTrench;
	}

	private static boolean isWithinDistance(
		Translation2d robotPosition,
		double maxShootingDistanceFromTargetMeters,
		String logPath,
		Translation2d targetTranslation
	) {
		boolean isWithinDistance = robotPosition.getDistance(targetTranslation) <= maxShootingDistanceFromTargetMeters;
		Logger.recordOutput(logPath + "/isInDistance", isWithinDistance);
		return isWithinDistance;
	}

	private static boolean isInAngleRange(
		Translation2d robotPosition,
		Rotation2d maxAngleFromCenter,
		String logPath,
		Translation2d targetTranslation,
		boolean isPass
	) {
		Rotation2d AngleBetweenRobotAndTarget = FieldMath.getRelativeTranslation(robotPosition, targetTranslation).getAngle();
		if (isPass) {
			AngleBetweenRobotAndTarget = FieldMath.getRelativeTranslation(targetTranslation, robotPosition).getAngle();
		}
		boolean isInAngleRange = Math.abs(AngleBetweenRobotAndTarget.getDegrees()) <= maxAngleFromCenter.getDegrees();
		Logger.recordOutput(logPath + "/isInRange", isInAngleRange);
		return isInAngleRange;
	}

	private static boolean isInAngleRangeToShoot(
		Translation2d robotPosition,
		Rotation2d maxAngleFromCenter,
		String logPath,
		Translation2d targetTranslation
	) {
		return isInAngleRange(robotPosition, maxAngleFromCenter, logPath, targetTranslation, false);
	}

	private static boolean isInAngleRangeToPass(
		Translation2d robotPosition,
		Rotation2d maxAngleFromCenter,
		String logPath,
		Translation2d targetTranslation
	) {
		return isInAngleRange(robotPosition, maxAngleFromCenter, logPath, targetTranslation, true);
	}

	private static boolean isTurretAtTargetPosition(Rotation2d turretPosition, Rotation2d targetTurretPosition, Rotation2d tolerance, String logPath) {
		boolean isAtHeading = MathUtil.isNear(targetTurretPosition.getDegrees(), turretPosition.getDegrees(), tolerance.getDegrees());
		Logger.recordOutput(logPath + "/isAtHeading", isAtHeading);
		return isAtHeading;
	}

	private static boolean isFlywheelAtVelocity(
		Rotation2d wantedFlywheelVelocityRPS,
		Rotation2d flywheelVelocityRPS,
		Rotation2d flywheelVelocityToleranceRPS,
		String logPath
	) {
		boolean isFlywheelAtVelocity = isNear(
			wantedFlywheelVelocityRPS.getDegrees(),
			flywheelVelocityRPS.getDegrees(),
			flywheelVelocityToleranceRPS.getDegrees()
		);
		Logger.recordOutput(logPath + "/isFlywheelAtVelocity", isFlywheelAtVelocity);
		return isFlywheelAtVelocity;
	}

	private static boolean isHoodAtPositon(Rotation2d targetHoodPosition, Rotation2d hoodPosition, Rotation2d tolerance, String logPath) {
		boolean isHoodAtPosition = MathUtil.isNear(targetHoodPosition.getDegrees(), hoodPosition.getDegrees(), tolerance.getDegrees());
		Logger.recordOutput(logPath + "/isHoodAtPositon", isHoodAtPosition);
		return isHoodAtPosition;
	}

	private static boolean isReadyToReleaseBalls(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromTargetCenter,
		double maxShootingDistanceFromTargetMeters,
		Translation2d targetTranslation,
		String actionLogPath,
		boolean isPass
	) {
		Pose2d robotPose = robot.getPoseEstimator().getEstimatedPose();
		String logPath = shootingChacksLogPath + "/IsReadyTo" + actionLogPath;
		Rotation2d flywheelVelocityRPS = robot.getFlyWheel().getVelocity();
		Rotation2d hoodPosition = robot.getHood().getPosition();

		boolean isWithinDistance = isWithinDistance(robotPose.getTranslation(), maxShootingDistanceFromTargetMeters, logPath, targetTranslation);

		boolean isInRange = isInAngleRangeToShoot(robotPose.getTranslation(), maxAngleFromTargetCenter, logPath, targetTranslation);
		if (isPass) {
			isInRange = isInAngleRangeToPass(robotPose.getTranslation(), maxAngleFromTargetCenter, logPath, targetTranslation);
		}
		boolean isAtTurretAtTarget = isTurretAtTargetPosition(
			robot.getTurret().getPosition(),
			ShootingCalculations.getShootingParams().targetTurretPosition(),
			headingTolerance,
			logPath
		);

		boolean isFlywheelReadyToShoot = isFlywheelAtVelocity(
			ShootingCalculations.getShootingParams().targetFlywheelVelocityRPS(),
			flywheelVelocityRPS,
			flywheelVelocityToleranceRPS,
			logPath
		);

		boolean isHoodAtPosition = isHoodAtPositon(
			ShootingCalculations.getShootingParams().targetHoodPosition(),
			hoodPosition,
			hoodPositionTolerance,
			logPath
		);

		return isFlywheelReadyToShoot
			&& isHoodAtPosition
			&& isInRange
			&& isWithinDistance
			&& isAtTurretAtTarget
			&& !isOnBumpOrUnderTrench(robotPose, logPath);
	}

	private static boolean canContinueReleasingBalls(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromTargetCenter,
		double maxShootingDistanceFromTargetMeters,
		Translation2d target,
		String actionLogPath,
		boolean isPass
	) {
		Pose2d robotPose = robot.getPoseEstimator().getEstimatedPose();
		String logPath = shootingChacksLogPath + "/CanContinue" + actionLogPath;

		Rotation2d flywheelVelocityRPS = robot.getFlyWheel().getVelocity();
		Rotation2d hoodPosition = robot.getHood().getPosition();

		boolean isWithinDistance = isWithinDistance(robotPose.getTranslation(), maxShootingDistanceFromTargetMeters, logPath, target);

		boolean isInRange = isInAngleRange(robotPose.getTranslation(), maxAngleFromTargetCenter, logPath, target, isPass);

		boolean isAtTurretAtTarget = isTurretAtTargetPosition(
			robot.getTurret().getPosition(),
			ShootingCalculations.getShootingParams().targetTurretPosition(),
			headingTolerance,
			logPath
		);

		boolean isFlywheelReadyToShoot = isFlywheelAtVelocity(
			ShootingCalculations.getShootingParams().targetFlywheelVelocityRPS(),
			flywheelVelocityRPS,
			flywheelVelocityToleranceRPS,
			logPath
		);

		boolean isHoodAtPosition = isHoodAtPositon(
			ShootingCalculations.getShootingParams().targetHoodPosition(),
			hoodPosition,
			hoodPositionTolerance,
			logPath
		);

		return isFlywheelReadyToShoot
			&& isHoodAtPosition
			&& isInRange
			&& isWithinDistance
			&& isAtTurretAtTarget
			&& !isOnBumpOrUnderTrench(robotPose, logPath);
	}

	private static boolean calibrationIsReadyToReleaseBalls(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		String actionLogPath
	) {
		String logPath = shootingChacksLogPath + "/calibrationIsReadyTo" + actionLogPath;

		Rotation2d flywheelVelocityRPS = robot.getFlyWheel().getVelocity();
		Rotation2d hoodPosition = robot.getHood().getPosition();

		boolean isFlywheelReadyToShoot = isFlywheelAtVelocity(
			ShooterConstants.flywheelCalibrationRotations.get(),
			flywheelVelocityRPS,
			flywheelVelocityToleranceRPS,
			logPath
		);

		boolean isHoodAtPosition = isHoodAtPositon(ShooterConstants.hoodCalibrationAngle.get(), hoodPosition, hoodPositionTolerance, logPath);

		return isFlywheelReadyToShoot && isHoodAtPosition;
	}

	public static boolean isReadyToShoot(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromHubCenter,
		double maxShootingDistanceFromTargetMeters
	) {
		return isReadyToReleaseBalls(
			robot,
			flywheelVelocityToleranceRPS,
			hoodPositionTolerance,
			headingTolerance,
			maxAngleFromHubCenter,
			maxShootingDistanceFromTargetMeters,
			Field.getHubMiddle(),
			"Shoot",
			false
		);
	}

	public static boolean isReadyToPass(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromTargetCenter,
		double maxShootingDistanceFromTargetMeters
	) {
		return isReadyToReleaseBalls(
			robot,
			flywheelVelocityToleranceRPS,
			hoodPositionTolerance,
			headingTolerance,
			maxAngleFromTargetCenter,
			maxShootingDistanceFromTargetMeters,
			ShootingCalculations.getShootingParams().targetLandingPosition(),
			"Pass",
			true
		) && isInPositionForPassing(robot.getPoseEstimator().getEstimatedPose(), shootingChacksLogPath + "/IsReadyToPass");
	}

	public static boolean canContinueShooting(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromHubCenter,
		double maxShootingDistanceFromTargetMeters
	) {
		return canContinueReleasingBalls(
			robot,
			flywheelVelocityToleranceRPS,
			hoodPositionTolerance,
			headingTolerance,
			maxAngleFromHubCenter,
			maxShootingDistanceFromTargetMeters,
			Field.getHubMiddle(),
			"Shooting",
			false
		) && isInAllianceZone(robot.getPoseEstimator().getEstimatedPose().getTranslation());
	}

	public static boolean canContinuePassing(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromTargetCenter,
		double maxShootingDistanceFromTargetMeters
	) {
		return canContinueReleasingBalls(
			robot,
			flywheelVelocityToleranceRPS,
			hoodPositionTolerance,
			headingTolerance,
			maxAngleFromTargetCenter,
			maxShootingDistanceFromTargetMeters,
			ShootingCalculations.getShootingParams().targetLandingPosition(),
			"Passing",
			true
		)
			&& !isInAllianceZone(robot.getPoseEstimator().getEstimatedPose().getTranslation())
			&& isInPositionForPassing(robot.getPoseEstimator().getEstimatedPose(), shootingChacksLogPath + "canContinuePassing");
	}

	public static boolean calibrationIsReadyToShoot(Robot robot, Rotation2d flywheelVelocityToleranceRPS, Rotation2d hoodPositionTolerance) {
		return calibrationIsReadyToReleaseBalls(robot, flywheelVelocityToleranceRPS, hoodPositionTolerance, "Shoot");
	}

	public static boolean calibrationIsReadyToPass(Robot robot, Rotation2d flywheelVelocityToleranceRPS, Rotation2d hoodPositionTolerance) {
		return calibrationIsReadyToReleaseBalls(robot, flywheelVelocityToleranceRPS, hoodPositionTolerance, "Pass");
	}


}
