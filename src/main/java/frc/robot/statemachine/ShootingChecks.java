package frc.robot.statemachine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.robot.Robot;
import frc.robot.statemachine.shooterstatehandler.ShooterConstants;
import frc.utils.HubUtil;
import frc.utils.math.FieldMath;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.MathUtil.isNear;

public class ShootingChecks {

	private static final String shootingChecksLogPath = "ShootingChecks";

	public static boolean isInAllianceZone(Translation2d position) {
		if (Field.isFieldConventionAlliance()) {
			return position.getX() <= Field.ALLIANCE_START_LINE_X_VALUE;
		}
		return position.getX() >= FieldMath.mirrorX(Field.ALLIANCE_START_LINE_X_VALUE);
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
		Translation2d targetTranslation
	) {
		boolean isWithinDistance = robotPosition.getDistance(targetTranslation) <= maxShootingDistanceFromTargetMeters;
		return isWithinDistance;
	}

	private static boolean isTurretAtTargetPosition(Rotation2d turretPosition, Rotation2d targetTurretPosition, Rotation2d turretTolerance) {
		return MathUtil.isNear(targetTurretPosition.getDegrees(), turretPosition.getDegrees(), turretTolerance.getDegrees());
	}

	private static boolean isFlywheelAtVelocity(
		Rotation2d wantedFlywheelVelocityRPS,
		Rotation2d flywheelVelocityRPS,
		Rotation2d flywheelVelocityToleranceRPS
	) {
		boolean isFlywheelAtVelocity = isNear(
			wantedFlywheelVelocityRPS.getDegrees(),
			flywheelVelocityRPS.getDegrees(),
			flywheelVelocityToleranceRPS.getDegrees()
		);
		return isFlywheelAtVelocity;
	}

	private static boolean isHoodAtPositon(Rotation2d targetHoodPosition, Rotation2d hoodPosition, Rotation2d hoodTolerance) {
		return MathUtil.isNear(targetHoodPosition.getDegrees(), hoodPosition.getDegrees(), hoodTolerance.getDegrees());
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

		boolean isWithinDistance = isWithinDistance(predictedTurretPosition, maxShootingDistanceFromTargetMeters, targetTranslation);

		boolean isAtTurretAtTarget = isTurretAtTargetPosition(
			robot.getTurret().getPosition(),
			ShootingCalculations.getShootingParams().targetTurretPosition(),
			turretTolerance
		);

		boolean isFlywheelReadyToShoot = isFlywheelAtVelocity(
			ShootingCalculations.getShootingParams().targetFlywheelVelocityRPS(),
			robot.getFlyWheel().getVelocity(),
			flywheelVelocityToleranceRPS
		);

		boolean isHoodAtPosition = isHoodAtPositon(
			ShootingCalculations.getShootingParams().targetHoodPosition(),
			robot.getHood().getPosition(),
			hoodPositionTolerance
		);

		Logger.recordOutput(logPath + "/isInDistance", isWithinDistance);
		Logger.recordOutput(logPath + "/isTurretAtPosition", isAtTurretAtTarget);
		Logger.recordOutput(logPath + "/isFlywheelAtVelocity", isFlywheelReadyToShoot);
		Logger.recordOutput(logPath + "/isHoodAtPositon", isHoodAtPosition);

		return isFlywheelReadyToShoot && isHoodAtPosition && isWithinDistance && isAtTurretAtTarget
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

		boolean isWithinDistance = isWithinDistance(predictedTurretPosition, maxShootingDistanceFromTargetMeters, target);

		boolean isAtTurretAtTarget = isTurretAtTargetPosition(
			robot.getTurret().getPosition(),
			ShootingCalculations.getShootingParams().targetTurretPosition(),
			turretTolerance
		);

		boolean isFlywheelReadyToShoot = isFlywheelAtVelocity(
			ShootingCalculations.getShootingParams().targetFlywheelVelocityRPS(),
			robot.getFlyWheel().getVelocity(),
			flywheelVelocityToleranceRPS
		);

		boolean isHoodAtPosition = isHoodAtPositon(
			ShootingCalculations.getShootingParams().targetHoodPosition(),
			robot.getHood().getPosition(),
			hoodPositionTolerance
		);

		Logger.recordOutput(logPath + "/IsWithinDistance", isWithinDistance);
		Logger.recordOutput(logPath + "/IsTurretAtPosition", isAtTurretAtTarget);
		Logger.recordOutput(logPath + "/IsFlywheelAtVelocity", isFlywheelReadyToShoot);
		Logger.recordOutput(logPath + "/IsHoodAtPosition", isHoodAtPosition);

		return isFlywheelReadyToShoot && isHoodAtPosition && isWithinDistance && isAtTurretAtTarget /* && isPoseReliable */;
	}

	private static boolean calibrationIsReadyToShoot(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		String actionLogPath
	) {
		String logPath = shootingChecksLogPath + "/CalibrationIsReadyTo" + actionLogPath;

		boolean isFlywheelReadyToShoot = isFlywheelAtVelocity(
			ShooterConstants.flywheelCalibrationRotations.get(),
			robot.getFlyWheel().getVelocity(),
			flywheelVelocityToleranceRPS
		);

		boolean isHoodAtPosition = isHoodAtPositon(
			ShooterConstants.hoodCalibrationAngle.get(),
			robot.getHood().getPosition(),
			hoodPositionTolerance
		);

		Logger.recordOutput(logPath + "/IsFlywheelAtVelocity", isFlywheelReadyToShoot);
		Logger.recordOutput(logPath + "/IsHoodAtPosition", isHoodAtPosition);

		return isFlywheelReadyToShoot && isHoodAtPosition;
	}

	private static boolean calibrationCanContinueShooting(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		String actionLogPath
	) {
		String logPath = shootingChecksLogPath + "/CalibrationCanContinue" + actionLogPath;

		boolean isFlywheelReadyToShoot = isFlywheelAtVelocity(
			ShooterConstants.flywheelCalibrationRotations.get(),
			robot.getFlyWheel().getVelocity(),
			flywheelVelocityToleranceRPS
		);

		boolean isHoodAtPosition = isHoodAtPositon(
			ShooterConstants.hoodCalibrationAngle.get(),
			robot.getHood().getPosition(),
			hoodPositionTolerance
		);

		Logger.recordOutput(logPath + "/IsFlywheelAtVelocity", isFlywheelReadyToShoot);
		Logger.recordOutput(logPath + "/IsHoodAtPosition", isHoodAtPosition);

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
			"Shoot"
		);
		boolean isOurHubReadyToStartShooting = isHubReadyToStartShooting(
			ShootingCalculations.getDistanceFromHub(ShootingCalculations.getShootingParams().predictedTurretPoseWhenBallLands())
		);
		boolean isInAllianceZone = isInAllianceZone(robot.getPoseEstimator().getEstimatedPose().getTranslation());

		Logger.recordOutput(shootingChecksLogPath + "/IsReadyToScore/IsOurHubReadyToStartShooting", isOurHubReadyToStartShooting);
		Logger.recordOutput(shootingChecksLogPath + "/IsReadyToScore/IsInAllianceZone", isInAllianceZone);

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
			ShootingCalculations.getFieldRelativeTurretPosition(
				new Pose2d(ShootingCalculations.getShootingParams().predictedTurretPoseWhenBallLands(), robot.getTurret().getPosition())
			),
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
		boolean isInAllianceZone = isInAllianceZone(robot.getPoseEstimator().getEstimatedPose().getTranslation());
		boolean isOurHubReadyToStartShooting = isHubReadyToStartShooting(
			ShootingCalculations.getDistanceFromHub(ShootingCalculations.getShootingParams().predictedTurretPoseWhenBallLands())
		);
		return canContinueShooting && isInAllianceZone && isOurHubReadyToStartShooting;
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
			ShootingCalculations.getFieldRelativeTurretPosition(
				new Pose2d(ShootingCalculations.getShootingParams().predictedTurretPoseWhenBallLands(), robot.getTurret().getPosition())
			),
			shootingChecksLogPath + "/CanContinuePassing"
		);
		return canContinuePassing && isInPositionForPassing;
	}

	public static boolean calibrationIsReadyToScore(Robot robot, Rotation2d flywheelVelocityToleranceRPS, Rotation2d hoodPositionTolerance) {
		return calibrationIsReadyToShoot(robot, flywheelVelocityToleranceRPS, hoodPositionTolerance, "Score");
	}

	public static boolean calibrationIsReadyToPass(Robot robot, Rotation2d flywheelVelocityToleranceRPS, Rotation2d hoodPositionTolerance) {
		return calibrationIsReadyToShoot(robot, flywheelVelocityToleranceRPS, hoodPositionTolerance, "Pass");
	}

	public static boolean calibrationCanContinueScoring(Robot robot, Rotation2d flywheelVelocityToleranceRPS, Rotation2d hoodPositionTolerance) {
		return calibrationCanContinueShooting(robot, flywheelVelocityToleranceRPS, hoodPositionTolerance, "Scoring");
	}

	public static boolean calibrationCanContinuePassing(Robot robot, Rotation2d flywheelVelocityToleranceRPS, Rotation2d hoodPositionTolerance) {
		return calibrationCanContinueShooting(robot, flywheelVelocityToleranceRPS, hoodPositionTolerance, "Passing");
	}

	public static boolean isHubReadyToStartShooting(double distanceFromHubMeters) {
		return HubUtil
			.isOurHubActive(TimeUtil.getTimeSinceTeleopInitSeconds() + ShootingCalculations.getDistanceToBallFlightTime(distanceFromHubMeters));
	}

}
