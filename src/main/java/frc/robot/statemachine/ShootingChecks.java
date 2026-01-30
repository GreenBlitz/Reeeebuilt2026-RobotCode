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

	private static final String shootingChacksLogPath = "Statemachine/ShootingChecks";

	public static boolean isInAllianceZone(Translation2d position) {
		if (Field.isFieldConventionAlliance()) {
			return position.getX() < Field.getHubMiddle().getX();
		}
		return position.getX() > Field.getHubMiddle().getX();
	}

	public static boolean isInPassingAreaOfDenial(Pose2d robotPose) {
		Translation2d turretPosition = ShootingCalculations.getFieldRelativeTurretPosition(robotPose);
		return (turretPosition.getY() < ShooterConstants.MAX_Y_FOR_PASSING_AREA_OF_DANIEL
			&& turretPosition.getY() > ShooterConstants.MIN_Y_FOR_PASSING_AREA_OF_DANIEL);
	}

	public static boolean isOnBumpOrUnderTrench(Pose2d robotPose) {
		Translation2d turretPosition = ShootingCalculations.getFieldRelativeTurretPosition(robotPose);
		Translation2d blueAllianceRelativeTurretPosition = Field.getAllianceRelative(turretPosition);
		return (blueAllianceRelativeTurretPosition.getX() < Field.OUTPOST_TRENCH_MIDDLE.getX() + (Field.TRENCH_X_AXIS_LENGTH_METERS / 2)
			&& blueAllianceRelativeTurretPosition.getX()
				> ShooterConstants.MIN_Y_FOR_PASSING_AREA_OF_DANIEL - (Field.TRENCH_X_AXIS_LENGTH_METERS / 2));
	}

	private static boolean isWithinDistance(
		Translation2d robotPosition,
		double maxShootingDistanceFromTargetMeters,
		String logPath,
		Translation2d target
	) {
		boolean isWithinDistance = robotPosition.getDistance(target) <= maxShootingDistanceFromTargetMeters;
		Logger.recordOutput(logPath + "/isInDistance", isWithinDistance);
		return isWithinDistance;
	}

	private static boolean isInAngleRange(Translation2d robotPosition, Rotation2d maxAngleFromCenter, String logPath, Translation2d Target) {
		Rotation2d AngleBetweenRobotAndTarget = FieldMath.getRelativeTranslation(Target, robotPosition).getAngle();
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

	public static boolean isReadyToShoot(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromHubCenter,
		double maxShootingDistanceFromTargetMeters
	) {
		Translation2d target;
		String logPath;
		Pose2d robotPose = robot.getPoseEstimator().getEstimatedPose();
		boolean isInAllianceZone = isInAllianceZone(robotPose.getTranslation());
		if (isInAllianceZone) {
			target = Field.getHubMiddle();
			logPath = shootingChacksLogPath + "/IsReadyToShootAtHub";
		} else {
			target = ShootingCalculations.getOptimalPassingPosition(robotPose);
			logPath = shootingChacksLogPath + "/IsReadyToPass";
		}
		Rotation2d flywheelVelocityRPS = robot.getFlyWheel().getVelocity();
		Rotation2d hoodPosition = robot.getHood().getPosition();

		boolean isWithinDistance = isWithinDistance(robotPose.getTranslation(), maxShootingDistanceFromTargetMeters, logPath, target);

		boolean isInRange = isInAngleRange(robotPose.getTranslation(), maxAngleFromHubCenter, logPath, target);

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

		return isFlywheelReadyToShoot
			&& isHoodAtPosition
			&& isInRange
			&& isWithinDistance
			&& isAtTurretAtTarget
			&& !isOnBumpOrUnderTrench(robotPose)
			&& (isInAllianceZone || !isInPassingAreaOfDenial(robotPose));
	}

	public static boolean canContinueShooting(
		Robot robot,
		Rotation2d flywheelVelocityToleranceRPS,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromHubCenter,
		double maxShootingDistanceFromTargetMeters
	) {
		Pose2d robotPose = robot.getPoseEstimator().getEstimatedPose();
		String logPath;
		Translation2d target;
		boolean isInAllianceZone = isInAllianceZone(robotPose.getTranslation());
		if (isInAllianceZone) {
			logPath = shootingChacksLogPath + "/CanContinueShootingAtHub";
			target = Field.getHubMiddle();
		} else {
			logPath = shootingChacksLogPath + "/CanContinuePassing";
			target = ShootingCalculations.getOptimalPassingPosition(robotPose);
			Logger.recordOutput(logPath + "targetPassingPosition", target);
		}
		Rotation2d flywheelVelocityRPS = robot.getFlyWheel().getVelocity();
		Rotation2d hoodPosition = robot.getHood().getPosition();

		boolean isWithinDistance = isWithinDistance(robotPose.getTranslation(), maxShootingDistanceFromTargetMeters, logPath, target);

		boolean isInRange = isInAngleRange(robotPose.getTranslation(), maxAngleFromHubCenter, logPath, target);

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

		return isFlywheelReadyToShoot
			&& isHoodAtPosition
			&& isInRange
			&& isWithinDistance
			&& isAtTurretAtTarget
			&& isOnBumpOrUnderTrench(robotPose)
			&& (isInAllianceZone || !isInPassingAreaOfDenial(robotPose));
	}

	public static boolean calibrationIsReadyToShoot(Robot robot, Rotation2d flywheelVelocityToleranceRPS, Rotation2d hoodPositionTolerance) {
		String logPath;
		if (isInAllianceZone(robot.getPoseEstimator().getEstimatedPose().getTranslation())) {
			logPath = shootingChacksLogPath + "/calibrationIsReadyToShootAtHub";
		} else {
			logPath = shootingChacksLogPath + "/calibrationIsReadyToPass";
		}
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

}
