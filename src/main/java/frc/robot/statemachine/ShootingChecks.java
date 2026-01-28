package frc.robot.statemachine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.robot.Robot;
import frc.robot.statemachine.shooterstatehandler.ShooterConstants;
import frc.utils.driverstation.DriverStationUtil;
import frc.utils.math.FieldMath;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.MathUtil.isNear;

public class ShootingChecks {

	private static final String shootingChacksLogPath = "Statemachine/ShootingChecks";

	public static boolean isInAllianceZone(Translation2d position) {
		if (DriverStationUtil.isBlueAlliance()) {
			return position.getX() < Field.getHubMiddle().getX();
		}
		return position.getX() > Field.getHubMiddle().getX();
	}

	private static boolean isWithinDistance(
		Translation2d robotPosition,
		double maxShootingDistanceFromTargetMeters,
		String logPath,
		Translation2d passingTarget
	) {
		boolean isWithinDistance = robotPosition.getDistance(passingTarget) <= maxShootingDistanceFromTargetMeters;
		Logger.recordOutput(logPath + "/isInDistance", isWithinDistance);
		return isWithinDistance;
	}

	private static boolean isInAngleRange(
		Translation2d robotPosition,
		Rotation2d maxAngleFromCenter,
		String logPath,
		Translation2d passingTarget
	) {
		Rotation2d AngleBetweenRobotAndTarget = FieldMath.getRelativeTranslation(passingTarget, robotPosition).getAngle();
		boolean isInAngleRange = Math.abs(AngleBetweenRobotAndTarget.getDegrees()) <= maxAngleFromCenter.getDegrees();
		Logger.recordOutput(logPath + "/isInRange", isInAngleRange);
		return isInAngleRange;
	}

	private static boolean isTurretAtTarget(Rotation2d turretPosition, Rotation2d target, Rotation2d tolerance, String logPath) {
		boolean isAtHeading = MathUtil.isNear(target.getDegrees(), turretPosition.getDegrees(), tolerance.getDegrees());
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

	private static boolean isHoodAtPositon(Rotation2d wantedPosition, Rotation2d hoodPosition, Rotation2d tolerance, String logPath) {
		boolean isHoodAtPosition = MathUtil.isNear(wantedPosition.getDegrees(), hoodPosition.getDegrees(), tolerance.getDegrees());
		Logger.recordOutput(logPath + "/isHoodAtPositon", isHoodAtPosition);
		return isHoodAtPosition;
	}

	private static boolean isReadyToShoot(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromHubCenter,
		double maxShootingDistanceFromTargetMeters,
		Translation2d passingTarget,
		String actionLogPath
	) {
		String logPath = shootingChacksLogPath + "/IsReadyTo" + actionLogPath;
		Pose2d robotPose = robot.getPoseEstimator().getEstimatedPose();
		Rotation2d flywheelVelocityRPS = robot.getFlyWheel().getVelocity();
		Rotation2d hoodPosition = robot.getHood().getPosition();

		boolean isWithinDistance = isWithinDistance(robotPose.getTranslation(), maxShootingDistanceFromTargetMeters, logPath, passingTarget);

		boolean isInRange = isInAngleRange(robotPose.getTranslation(), maxAngleFromHubCenter, logPath, passingTarget);

		boolean isAtTurretAtTarget = isTurretAtTarget(
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
		Rotation2d maxAngleFromHubCenter,
		double maxShootingDistanceFromTargetMeters,
		Translation2d passingTarget,
		String actionLogPath

	) {
		String logPath = shootingChacksLogPath + "/CanContinue" + actionLogPath;
		Pose2d robotPose = robot.getPoseEstimator().getEstimatedPose();
		Rotation2d flywheelVelocityRPS = robot.getFlyWheel().getVelocity();
		Rotation2d hoodPosition = robot.getHood().getPosition();

		boolean isWithinDistance = isWithinDistance(robotPose.getTranslation(), maxShootingDistanceFromTargetMeters, logPath, passingTarget);

		boolean isInRange = isInAngleRange(robotPose.getTranslation(), maxAngleFromHubCenter, logPath, passingTarget);

		boolean isAtTurretAtTarget = isTurretAtTarget(
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

	private static boolean calibrationIsReadyToShoot(
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

	public static boolean isReadyToShootAtHub(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromHubCenter,
		double maxShootingDistanceFromTargetMeters
	) {
		String logPath = "ShootAtHub";
		return isReadyToShoot(
			robot,
			flywheelVelocityToleranceRPS,
			hoodPositionTolerance,
			headingTolerance,
			maxAngleFromHubCenter,
			maxShootingDistanceFromTargetMeters,
			Field.getHubMiddle(),
			logPath
		);
	}

	public static boolean canContinueShootingAtHub(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromHubCenter,
		double maxShootingDistanceFromTargetMeters
	) {
		String logPath = "ShootingAtHub";
		return canContinueShooting(
			robot,
			flywheelVelocityToleranceRPS,
			hoodPositionTolerance,
			headingTolerance,
			maxAngleFromHubCenter,
			maxShootingDistanceFromTargetMeters,
			Field.getHubMiddle(),
			logPath
		);
	}

	public static boolean calibrationIsReadyToShootAtHub(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance
	) {
		String logPath = "ShootAtHub";
		return calibrationIsReadyToShoot(robot, flywheelVelocityToleranceRPS, hoodPositionTolerance, logPath);
	}

	public static boolean isReadyToPass(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromHubCenter,
		double maxShootingDistanceFromTargetMeters
	) {
		String logPath = "Pass";
		return isReadyToShoot(
			robot,
			flywheelVelocityToleranceRPS,
			hoodPositionTolerance,
			headingTolerance,
			maxAngleFromHubCenter,
			maxShootingDistanceFromTargetMeters,
			Field.getHubMiddle(),
			logPath
		);
	}

	public static boolean canContinuePassing(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromHubCenter,
		double maxShootingDistanceFromTargetMeters
	) {
		String logPath = "Passing";
		return canContinueShooting(
			robot,
			flywheelVelocityToleranceRPS,
			hoodPositionTolerance,
			headingTolerance,
			maxAngleFromHubCenter,
			maxShootingDistanceFromTargetMeters,
			Field.getHubMiddle(),
			logPath
		);
	}

	public static boolean calibrationIsReadyToPass(Robot robot, Rotation2d flywheelVelocityToleranceRPS, Rotation2d hoodPositionTolerance) {
		String logPath = "Pass";
		return calibrationIsReadyToShoot(robot, flywheelVelocityToleranceRPS, hoodPositionTolerance, logPath);
	}

}
