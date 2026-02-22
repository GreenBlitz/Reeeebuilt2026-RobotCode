package frc.robot.statemachine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.robot.Robot;
import frc.robot.statemachine.shooterstatehandler.ShooterConstants;
import frc.utils.GamePeriodUtils;
import frc.utils.HubUtil;
import frc.utils.math.FieldMath;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.MathUtil.isNear;

public class ShootingChecks {

	private static final String shootingChecksLogPath = "ShootingChecks";

	public static boolean isInAllianceZone(Translation2d position) {
		boolean isPositionInAllianceZone = Field.getAllianceRelative(position).getX() <= Field.ALLIANCE_START_LINE_X_VALUE;
		Logger.recordOutput(shootingChecksLogPath + "/IsInAllianceZone", isPositionInAllianceZone);
		return isPositionInAllianceZone;
	}

	public static boolean isBehindHub(Translation2d turretTranslation) {
		return turretTranslation.getY() < Field.MAX_HUB_Y_VALUE && turretTranslation.getY() > Field.MIN_HUB_Y_VALUE;
	}

	private static boolean isInPositionForPassing(Translation2d turretTranslation, String logPath) {
		Translation2d allianceRelativeTurretTranslation = Field.getAllianceRelative(turretTranslation);
		boolean isBehindHub = isBehindHub(turretTranslation);
		boolean isFarEnoughBehindHub = allianceRelativeTurretTranslation.getX() > StateMachineConstants.getMinXValueForBehindHubPassing();
		if (!Field.isFieldConventionAlliance()) {
			isFarEnoughBehindHub = allianceRelativeTurretTranslation.getX()
				> FieldMath.mirrorX(StateMachineConstants.getMinXValueForBehindHubPassing());
		}
		Logger.recordOutput(logPath + "/IsBehindHub", isBehindHub);
		Logger.recordOutput(logPath + "/IsFarEnoughBehindHub", isFarEnoughBehindHub);
		return !isBehindHub || isFarEnoughBehindHub;
	}

	private static boolean isWithinDistance(
		Translation2d robotPosition,
		double maxShootingDistanceFromTargetMeters,
		Translation2d targetTranslation,
		String logPath
	) {
		boolean isWithinDistance = robotPosition.getDistance(targetTranslation) <= maxShootingDistanceFromTargetMeters;
		Logger.recordOutput(logPath + "/IsInDistance", isWithinDistance);
		return isWithinDistance;
	}

	private static boolean isTurretAtTargetPosition(
		Rotation2d turretPosition,
		Rotation2d targetTurretPosition,
		Rotation2d turretTolerance,
		String logPath
	) {
		boolean isTurretAtPosition = MathUtil
			.isNear(targetTurretPosition.getDegrees(), turretPosition.getDegrees(), turretTolerance.getDegrees());
		Logger.recordOutput(logPath + "/IsTurretAtPosition", isTurretAtPosition);
		return isTurretAtPosition;
	}

	private static boolean isFlywheelAtVelocity(
		Rotation2d flywheelVelocityRPS,
		Rotation2d targetFlywheelVelocityRPS,
		Rotation2d flywheelVelocityToleranceRPS,
		String logPath
	) {
		boolean isFlywheelAtVelocity = isNear(
			targetFlywheelVelocityRPS.getDegrees(),
			flywheelVelocityRPS.getDegrees(),
			flywheelVelocityToleranceRPS.getDegrees()
		);
		Logger.recordOutput(logPath + "/IsFlywheelAtVelocity", isFlywheelAtVelocity);
		return isFlywheelAtVelocity;
	}

	private static boolean isHoodAtPosition(Rotation2d hoodPosition, Rotation2d targetHoodPosition, Rotation2d hoodTolerance, String logPath) {
		boolean isHoodAtPosition = MathUtil.isNear(targetHoodPosition.getDegrees(), hoodPosition.getDegrees(), hoodTolerance.getDegrees());
		Logger.recordOutput(logPath + "/IsHoodAtPosition", isHoodAtPosition);
		return isHoodAtPosition;
	}

	private static boolean isReadyToShoot(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d turretTolerance,
		double maxShootingDistanceFromTargetMeters,
		Translation2d targetTranslation,
		String actionLogPath
	) {
		Translation2d predictedTurretPosition = ShootingCalculations.getShootingParams().predictedTurretPoseWhenBallLands();
		String logPath = shootingChecksLogPath + "/IsReadyTo" + actionLogPath;

		boolean isAtTurretAtTarget = isTurretAtTargetPosition(
			robot.getTurret().getPosition(),
			ShootingCalculations.getShootingParams().targetTurretPosition(),
			turretTolerance,
			logPath
		);

		boolean isFlywheelReadyToShoot = isFlywheelAtVelocity(
			robot.getFlyWheel().getVelocity(),
			ShootingCalculations.getShootingParams().targetFlywheelVelocityRPS(),
			flywheelVelocityToleranceRPS,
			logPath
		);

		boolean isHoodAtPosition = isHoodAtPosition(
			robot.getHood().getPosition(),
			ShootingCalculations.getShootingParams().targetHoodPosition(),
			hoodPositionTolerance,
			logPath
		);

		boolean isTurretWithinDistance = isWithinDistance(
			predictedTurretPosition,
			maxShootingDistanceFromTargetMeters,
			targetTranslation,
			logPath
		);

		return isAtTurretAtTarget && isFlywheelReadyToShoot && isHoodAtPosition && isTurretWithinDistance
		/* && isPoseReliable */;
	}

	private static boolean canContinueShooting(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d turretTolerance,
		double maxShootingDistanceFromTargetMeters,
		Translation2d target,
		String actionLogPath
	) {
		Translation2d predictedTurretPosition = ShootingCalculations.getShootingParams().predictedTurretPoseWhenBallLands();
		String logPath = shootingChecksLogPath + "/CanContinue" + actionLogPath;

		boolean isAtTurretAtTarget = isTurretAtTargetPosition(
			robot.getTurret().getPosition(),
			ShootingCalculations.getShootingParams().targetTurretPosition(),
			turretTolerance,
			logPath
		);

		boolean isFlywheelReadyToShoot = isFlywheelAtVelocity(
			robot.getFlyWheel().getVelocity(),
			ShootingCalculations.getShootingParams().targetFlywheelVelocityRPS(),
			flywheelVelocityToleranceRPS,
			logPath
		);

		boolean isHoodAtPosition = isHoodAtPosition(
			robot.getHood().getPosition(),
			ShootingCalculations.getShootingParams().targetHoodPosition(),
			hoodPositionTolerance,
			logPath
		);
		boolean isTurretWithinDistance = isWithinDistance(predictedTurretPosition, maxShootingDistanceFromTargetMeters, target, logPath);

		return isAtTurretAtTarget && isFlywheelReadyToShoot && isHoodAtPosition && isTurretWithinDistance /* && isPoseReliable */;
	}

	static boolean calibrationIsReadyToShoot(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		String actionLogPath
	) {
		String logPath = shootingChecksLogPath + "/CalibrationIsReadyTo" + actionLogPath;

		boolean isFlywheelReadyToShoot = isFlywheelAtVelocity(
			robot.getFlyWheel().getVelocity(),
			ShooterConstants.flywheelCalibrationRotations.get(),
			flywheelVelocityToleranceRPS,
			logPath
		);

		boolean isHoodAtPosition = isHoodAtPosition(
			robot.getHood().getPosition(),
			ShooterConstants.hoodCalibrationAngle.get(),
			hoodPositionTolerance,
			logPath
		);

		return isFlywheelReadyToShoot && isHoodAtPosition;
	}

	static boolean calibrationCanContinueShooting(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		String actionLogPath
	) {
		String logPath = shootingChecksLogPath + "/CalibrationCanContinue" + actionLogPath;

		boolean isFlywheelReadyToShoot = isFlywheelAtVelocity(
			robot.getFlyWheel().getVelocity(),
			ShooterConstants.flywheelCalibrationRotations.get(),
			flywheelVelocityToleranceRPS,
			logPath
		);

		boolean isHoodAtPosition = isHoodAtPosition(
			robot.getHood().getPosition(),
			ShooterConstants.hoodCalibrationAngle.get(),
			hoodPositionTolerance,
			logPath
		);

		return isFlywheelReadyToShoot && isHoodAtPosition;
	}

	public static boolean isReadyToScore(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d turretTolerance,
		double maxShootingDistanceFromTargetMeters
	) {
		boolean isReadyToShoot = isReadyToShoot(
			robot,
			flywheelVelocityToleranceRPS,
			hoodPositionTolerance,
			turretTolerance,
			maxShootingDistanceFromTargetMeters,
			Field.getHubMiddle(),
			"Score"
		);
		boolean isOurHubReadyToStartShooting = isOurHubReadyToStartShooting(
			ShootingCalculations.getDistanceFromHub(ShootingCalculations.getShootingParams().predictedTurretPoseWhenBallLands())
		);
		boolean isInAllianceZone = isInAllianceZone(robot.getPoseEstimator().getEstimatedPose().getTranslation());

		return isReadyToShoot && isOurHubReadyToStartShooting && isInAllianceZone;
	}

	public static boolean isReadyToPass(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d turretTolerance,
		double maxShootingDistanceFromTargetMeters
	) {
		boolean isReadyToShoot = isReadyToShoot(
			robot,
			flywheelVelocityToleranceRPS,
			hoodPositionTolerance,
			turretTolerance,
			maxShootingDistanceFromTargetMeters,
			ShootingCalculations.getShootingParams().targetLandingPosition(),
			"Pass"
		);
		boolean isInPositionForPassing = isInPositionForPassing(
			ShootingCalculations.getShootingParams().predictedTurretPoseWhenBallLands(),
			shootingChecksLogPath + "/IsReadyToPass"
		);
		return isReadyToShoot && isInPositionForPassing;
	}

	public static boolean canContinueScoring(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d turretTolerance,
		double maxShootingDistanceFromTargetMeters
	) {
		boolean canContinueShooting = canContinueShooting(
			robot,
			flywheelVelocityToleranceRPS,
			hoodPositionTolerance,
			turretTolerance,
			maxShootingDistanceFromTargetMeters,
			Field.getHubMiddle(),
			"Scoring"
		);

		boolean isOurHubReadyToStartShooting = isOurHubReadyToStartShooting(
			ShootingCalculations.getDistanceFromHub(ShootingCalculations.getShootingParams().predictedTurretPoseWhenBallLands())
		);

		boolean isInAllianceZone = isInAllianceZone(robot.getPoseEstimator().getEstimatedPose().getTranslation());

		return canContinueShooting && isOurHubReadyToStartShooting && isInAllianceZone;
	}

	public static boolean canContinuePassing(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		double maxShootingDistanceFromTargetMeters
	) {
		boolean canContinuePassing = canContinueShooting(
			robot,
			flywheelVelocityToleranceRPS,
			hoodPositionTolerance,
			headingTolerance,
			maxShootingDistanceFromTargetMeters,
			ShootingCalculations.getShootingParams().targetLandingPosition(),
			"Passing"
		);
		boolean isInPositionForPassing = isInPositionForPassing(
			ShootingCalculations.getShootingParams().predictedTurretPoseWhenBallLands(),
			shootingChecksLogPath + "/CanContinuePassing"
		);
		return canContinuePassing && isInPositionForPassing;
	}

	public static boolean isOurHubReadyToStartShooting(double distanceFromHubMeters) {
		boolean isOurHubReadyToStartShooting = HubUtil.isOurHubActive(
			TimeUtil.getTimeSinceTeleopInitSeconds()
				+ ShootingCalculations.getDistanceToBallFlightTime(distanceFromHubMeters)
				+ GamePeriodUtils.GAME_DURATION_SECONDS
		);
		Logger.recordOutput(shootingChecksLogPath + "/IsOurHubActiveToShoot", isOurHubReadyToStartShooting);
		return isOurHubReadyToStartShooting;
	}

}
