package frc.robot.statemachine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.robot.Robot;
import frc.robot.statemachine.shooterstatehandler.ShooterConstants;
import frc.robot.statemachine.superstructure.Superstructure;
import frc.utils.math.FieldMath;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.MathUtil.isNear;

public class ShootingChecks {

	private final Superstructure superstructure;
	private static final String isReadyToShootLogPath = "Statemachine/ShootingChecks/IsReadyToShoot";
	private static final String canContinueShootingLogPath = "Statemachine/ShootingChecks/canContinueShooting";

	public ShootingChecks(Superstructure superstructure) {
		this.superstructure = superstructure;
	}

	private static boolean isWithinDistance(Translation2d robotPosition, double maxShootingDistanceFromTargetMeters, String logPath) {
		boolean isWithinDistance = robotPosition.getDistance(Field.getHubMiddle()) <= maxShootingDistanceFromTargetMeters;
		Logger.recordOutput(logPath + "/isInDistance", isWithinDistance);
		return isWithinDistance;
	}

	private static boolean isInAngleRange(Translation2d robotPosition, Rotation2d maxAngleFromCenter, String logPath) {
		Rotation2d AngleBetweenRobotAndGoal = FieldMath.getRelativeTranslation(Field.getHubMiddle(), robotPosition).getAngle();
		boolean isInAngleRange = Math.abs(AngleBetweenRobotAndGoal.getDegrees()) <= maxAngleFromCenter.getDegrees();
		Logger.recordOutput(logPath + "/isInRange", isInAngleRange);
		return isInAngleRange;
	}

	private static boolean isTurretAtTarget(Rotation2d turretPosition, Rotation2d target, Rotation2d tolerance, String logPath) {
		boolean isAtHeading = MathUtil.isNear(target.getDegrees(), turretPosition.getDegrees(), tolerance.getDegrees());
		Logger.recordOutput(logPath + "/isAtHeading", isAtHeading);
		return isAtHeading;
	}

	private static boolean isFlywheelAtVelocity(
		Rotation2d wantedFlywheelVelocityRotation2dPerSecond,
		Rotation2d flywheelVelocityRotation2dPerSecond,
		Rotation2d flywheelVelocityToleranceRotation2dPerSecond,
		String logPath
	) {
		boolean isFlywheelAtVelocity = isNear(
			wantedFlywheelVelocityRotation2dPerSecond.getDegrees(),
			flywheelVelocityRotation2dPerSecond.getDegrees(),
			flywheelVelocityToleranceRotation2dPerSecond.getDegrees()
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
		Rotation2d flywheelVelocityToleranceRotation2dPerSecond,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromHubCenter,
		double maxShootingDistanceFromTargetMeters
	) {
		Pose2d robotPose = robot.getPoseEstimator().getEstimatedPose();
		Rotation2d flywheelVelocityRotation2dPerSecond = robot.getFlyWheel().getVelocity();
		Rotation2d hoodPosition = robot.getHood().getPosition();

		boolean isWithinDistance = isWithinDistance(robotPose.getTranslation(), maxShootingDistanceFromTargetMeters, isReadyToShootLogPath);

		boolean isInRange = isInAngleRange(robotPose.getTranslation(), maxAngleFromHubCenter, isReadyToShootLogPath);

		boolean isAtTurretAtTarget = isTurretAtTarget(
			robot.getTurret().getPosition(),
			ShootingCalculations.getShootingParams().targetTurretPosition(),
			headingTolerance,
			isReadyToShootLogPath
		);

		boolean isFlywheelReadyToShoot = isFlywheelAtVelocity(
			ShootingCalculations.getShootingParams().targetFlywheelVelocityRPS(),
			flywheelVelocityRotation2dPerSecond,
			flywheelVelocityToleranceRotation2dPerSecond,
			isReadyToShootLogPath
		);

		boolean isHoodAtPosition = isHoodAtPositon(
			ShootingCalculations.getShootingParams().targetHoodPosition(),
			hoodPosition,
			hoodPositionTolerance,
			isReadyToShootLogPath
		);

		return isFlywheelReadyToShoot && isHoodAtPosition && isInRange && isWithinDistance && isAtTurretAtTarget;
	}

	public static boolean canContinueShooting(
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

		boolean isWithinDistance = isWithinDistance(robotPose.getTranslation(), maxShootingDistanceFromTargetMeters, canContinueShootingLogPath);

		boolean isInRange = isInAngleRange(robotPose.getTranslation(), maxAngleFromHubCenter, canContinueShootingLogPath);

		boolean isAtTurretAtTarget = isTurretAtTarget(
			robot.getTurret().getPosition(),
			ShootingCalculations.getShootingParams().targetTurretPosition(),
			headingTolerance,
			canContinueShootingLogPath
		);

		boolean isFlywheelReadyToShoot = isFlywheelAtVelocity(
			ShootingCalculations.getShootingParams().targetFlywheelVelocityRPS(),
			flywheelVelocityRotation2dPerSecond,
			flywheelVelocityToleranceRotation2dPerSecond,
			canContinueShootingLogPath
		);

		boolean isHoodAtPosition = isHoodAtPositon(
			ShootingCalculations.getShootingParams().targetHoodPosition(),
			hoodPosition,
			hoodPositionTolerance,
			canContinueShootingLogPath
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
			flywheelVelocityToleranceRotation2dPerSecond,
			isReadyToShootLogPath
		);

		boolean isHoodAtPosition = isHoodAtPositon(
			ShooterConstants.hoodCalibrationAngle.get(),
			hoodPosition,
			hoodPositionTolerance,
			isReadyToShootLogPath
		);

		return isFlywheelReadyToShoot && isHoodAtPosition;
	}

}
