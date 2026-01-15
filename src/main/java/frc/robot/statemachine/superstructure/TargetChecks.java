package frc.robot.statemachine.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.statemachine.ShooterCalculations;
import frc.robot.statemachine.shooterstatehandler.ShooterConstants;
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

	private static boolean isWithinDistance(Translation2d robotPosition, Translation2d target, double maxShootingDistanceFromTargetMeters) {
		boolean isWithinDistance = robotPosition.getDistance(target) <= maxShootingDistanceFromTargetMeters;
		Logger.recordOutput(isReadyToShootLogPath + "/isInDistance", isWithinDistance);
		return isWithinDistance;
	}

	private static boolean isInAngleRange(Translation2d robotPosition, Translation2d target, Rotation2d maxAngleFromCenter) {
		Rotation2d AngleBetweenRobotAndGoal = FieldMath.getRelativeTranslation(target, robotPosition).getAngle();
		boolean isInAngleRange = Math.abs(AngleBetweenRobotAndGoal.getDegrees()) <= maxAngleFromCenter.getDegrees();
		Logger.recordOutput(isReadyToShootLogPath + "/isInRange", isInAngleRange);
		return isInAngleRange;
	}

	private static boolean isTurretAtTarget(Pose2d robotPose, Translation2d target, Arm turret, double tolerance) {
		Rotation2d wantedAngle = ShooterCalculations.getRobotRelativeLookAtTargetAngleForTurret(robotPose, turret.getPosition(), target);
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
		Rotation2d wantedFlywheelVelocityRotation2dPerSecond,
		Rotation2d flywheelVelocityToleranceRotation2dPerSecond,
		Rotation2d wantedHoodPosition,
		Rotation2d hoodPositionTolerance,
		Rotation2d headingTolerance,
		Rotation2d maxAngleFromHubCenter,
		double maxShootingDistanceFromTargetMeters
	) {
		Pose2d robotPose = robot.getPoseEstimator().getEstimatedPose();
		Rotation2d flywheelVelocityRotation2dPerSecond = robot.getFlyWheel().getVelocity();
		Rotation2d hoodPosition = robot.getHood().getPosition();
		Translation2d target = ShooterCalculations.getDesiredTargetInMotion(robot.getPoseEstimator().getEstimatedPose(), robot.getSwerve());
		boolean isWithinDistance = isWithinDistance(robotPose.getTranslation(), target, maxShootingDistanceFromTargetMeters);

		boolean isInRange = isInAngleRange(robotPose.getTranslation(), target, maxAngleFromHubCenter);

		boolean isAtHeading = isTurretAtTarget(robotPose, target, robot.getTurret(), headingTolerance.getDegrees());

		boolean isFlywheelReadyToShoot = isFlywheelAtVelocity(
			wantedFlywheelVelocityRotation2dPerSecond,
			flywheelVelocityRotation2dPerSecond,
			flywheelVelocityToleranceRotation2dPerSecond
		);

		boolean isHoodAtPosition = isHoodAtPositon(wantedHoodPosition, hoodPosition, hoodPositionTolerance);

		return isFlywheelReadyToShoot && isHoodAtPosition && isInRange && isWithinDistance && isAtHeading;
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
