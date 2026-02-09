package frc.robot.statemachine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.robot.Robot;
import frc.robot.statemachine.shooterstatehandler.ShooterConstants;
import frc.utils.math.FieldMath;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.MathUtil.isNear;

public class ShootingChecks {

	private static final String shootingChecksLogPath = "ShootingChecks";

	public static boolean isInAllianceZone(Translation2d position) {
		if (Field.isFieldConventionAlliance()) {
			return position.getX() < Field.getHubMiddle().getX();
		}
		return position.getX() > Field.getHubMiddle().getX();
	}

	public static boolean isBehindHub(Translation2d turretTranslation) {
		return turretTranslation.getY() < Field.MAX_HUB_Y_VALUE && turretTranslation.getY() > Field.MIN_HUB_Y_VALUE;
	}

	private static boolean isInPositionForPassing(Translation2d turretTranslation, String logPath) {
		Translation2d allianceRelativeTurretTranslation = Field.getAllianceRelative(turretTranslation);
		boolean isBehindHub = isBehindHub(turretTranslation);
		boolean isTooCloseToHub = allianceRelativeTurretTranslation.getX()
			> FieldMath.mirrorX(StateMachineConstants.getMinXValueForBehindHubPassing());
		Logger.recordOutput(logPath + "/IsBehindHub", isBehindHub);
		return !isBehindHub || !isTooCloseToHub;
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
		Translation2d allianceRelativeRobotPosition = Field.getAllianceRelative(robotPosition);
		Rotation2d AngleBetweenRobotAndTarget = FieldMath.getRelativeTranslation(allianceRelativeRobotPosition, targetTranslation).getAngle();
		if (Field.isFieldConventionAlliance() && isPass) {
			AngleBetweenRobotAndTarget = FieldMath.getRelativeTranslation(targetTranslation, allianceRelativeRobotPosition).getAngle();
		}
		boolean isInAngleRange = Math.abs(AngleBetweenRobotAndTarget.getDegrees()) <= maxAngleFromCenter.getDegrees();
		Logger.recordOutput(logPath + "/isInRange", isInAngleRange);
		return isInAngleRange;
	}

	private static boolean isInAngleRangeToShoot(
		Translation2d turretPosition,
		Rotation2d maxAngleFromCenter,
		String logPath,
		Translation2d targetTranslation
	) {
		return isInAngleRange(turretPosition, maxAngleFromCenter, logPath, targetTranslation, false);
	}

	private static boolean isInAngleRangeToPass(
		Translation2d TurretPosition,
		Rotation2d maxAngleFromCenter,
		String logPath,
		Translation2d targetTranslation
	) {
		return isInAngleRange(TurretPosition, maxAngleFromCenter, logPath, targetTranslation, true);
	}

	private static boolean isTurretAtTargetPosition(
		Rotation2d turretPosition,
		Rotation2d targetTurretPosition,
		Rotation2d tolerance,
		String logPath
	) {
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

	private static boolean isReadyToShoot(
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
		Translation2d turretPosition = ShootingCalculations.getShootingParams().predictedTurretPoseWhenBallLands();
		String logPath = shootingChecksLogPath + "/IsReadyTo" + actionLogPath;
		Rotation2d flywheelVelocityRPS = robot.getFlyWheel().getVelocity();
		Rotation2d hoodPosition = robot.getHood().getPosition();

		boolean isWithinDistance = isWithinDistance(turretPosition, maxShootingDistanceFromTargetMeters, logPath, targetTranslation);

		boolean isInRange = isInAngleRangeToShoot(turretPosition, maxAngleFromTargetCenter, logPath, targetTranslation);
		if (isPass) {
			isInRange = isInAngleRangeToPass(turretPosition, maxAngleFromTargetCenter, logPath, targetTranslation);
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

		return isFlywheelReadyToShoot && isHoodAtPosition && isInRange && isWithinDistance && isAtTurretAtTarget;
	}

	private static boolean canContinueShooting(
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
		Translation2d turretPosition = ShootingCalculations.getShootingParams().predictedTurretPoseWhenBallLands();
		String logPath = shootingChecksLogPath + "/CanContinue" + actionLogPath;

		Rotation2d flywheelVelocityRPS = robot.getFlyWheel().getVelocity();
		Rotation2d hoodPosition = robot.getHood().getPosition();

		boolean isWithinDistance = isWithinDistance(turretPosition, maxShootingDistanceFromTargetMeters, logPath, target);

		boolean isInRange = isInAngleRange(turretPosition, maxAngleFromTargetCenter, logPath, target, isPass);

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

		return isFlywheelReadyToShoot && isHoodAtPosition && isInRange && isWithinDistance && isAtTurretAtTarget;
	}

	private static boolean calibrationIsReadyToScore(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		String actionLogPath
	) {
		String logPath = shootingChecksLogPath + "/calibrationIsReadyTo" + actionLogPath;

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

	private static boolean calibrationCanContinueShooting(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		String actionLogPath
	) {
		String logPath = shootingChecksLogPath + "/calibrationIsReadyTo" + actionLogPath;

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

	public static boolean isReadyToScore(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromHubCenter,
		double maxShootingDistanceFromTargetMeters
	) {
		return isReadyToShoot(
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
		return isReadyToShoot(
			robot,
			flywheelVelocityToleranceRPS,
			hoodPositionTolerance,
			headingTolerance,
			maxAngleFromTargetCenter,
			maxShootingDistanceFromTargetMeters,
			ShootingCalculations.getShootingParams().targetLandingPosition(),
			"Pass",
			true
		)
			&& isInPositionForPassing(
				ShootingCalculations.getFieldRelativeTurretPosition(robot.getPoseEstimator().getEstimatedPose()),
				shootingChecksLogPath + "/IsReadyToPass"
			);
	}

	public static boolean canContinueScoring(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromHubCenter,
		double maxShootingDistanceFromTargetMeters
	) {
		return canContinueShooting(
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
		return canContinueShooting(
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
			&& isInPositionForPassing(
				ShootingCalculations.getFieldRelativeTurretPosition(robot.getPoseEstimator().getEstimatedPose()),
				shootingChecksLogPath + "/canContinuePassing"
			);
	}

	public static boolean calibrationIsReadyToScore(Robot robot, Rotation2d flywheelVelocityToleranceRPS, Rotation2d hoodPositionTolerance) {
		return calibrationIsReadyToScore(robot, flywheelVelocityToleranceRPS, hoodPositionTolerance, "Shoot");
	}

	public static boolean calibrationIsReadyToPass(Robot robot, Rotation2d flywheelVelocityToleranceRPS, Rotation2d hoodPositionTolerance) {
		return calibrationIsReadyToScore(robot, flywheelVelocityToleranceRPS, hoodPositionTolerance, "Pass");
	}

	public static boolean calibrationCanContinueScoring(Robot robot, Rotation2d flywheelVelocityToleranceRPS, Rotation2d hoodPositionTolerance) {
		return calibrationCanContinueShooting(robot, flywheelVelocityToleranceRPS, hoodPositionTolerance, "CalibrationScore");
	}

	public static boolean calibrationCanContinuePassing(Robot robot, Rotation2d flywheelVelocityToleranceRPS, Rotation2d hoodPositionTolerance) {
		return calibrationCanContinueShooting(robot, flywheelVelocityToleranceRPS, hoodPositionTolerance, "CalibrationPass");
	}


}
