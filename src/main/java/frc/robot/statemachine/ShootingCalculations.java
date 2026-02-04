package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.field.Field;
import frc.robot.statemachine.shooterstatehandler.ShootingParams;
import frc.robot.subsystems.constants.hood.HoodConstants;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.utils.InterpolationMap;
import org.littletonrobotics.junction.Logger;


public class ShootingCalculations {

	private static final String LOG_PATH = "ShootingCalculations";
	private static ShootingParams shootingParams = new ShootingParams(
		new Rotation2d(),
		HoodConstants.MINIMUM_POSITION,
		TurretConstants.MIN_POSITION,
		new Rotation2d(),
		new Translation2d(),
		Field.getHubMiddle()
	);

	public static ShootingParams getShootingParams() {
		return shootingParams;
	}

	private static ShootingParams calculateShootingParams(
		Pose2d robotPose,
		ChassisSpeeds fieldRelativeSpeeds,
		Rotation2d gyroYawAngularVelocity,
		InterpolationMap<Double, Rotation2d> hoodInterpolation,
		InterpolationMap<Double, Rotation2d> flywheelInterpolation,
		Translation2d targetTranslation
	) {
		// Calculate distance from turret to target
		Translation2d fieldRelativeTurretTranslation = getFieldRelativeTurretPosition(robotPose);
		double distanceFromTurretToTargetMeters = targetTranslation.getDistance(fieldRelativeTurretTranslation);
		// Split Robot's Speeds
		Translation2d robotTranslationalVelocity = new Translation2d(
			fieldRelativeSpeeds.vxMetersPerSecond,
			fieldRelativeSpeeds.vyMetersPerSecond
		);

		// Turret Field Relative Velocity
		Translation2d turretTangentialVelocity = TurretConstants.TURRET_POSITION_RELATIVE_TO_ROBOT.toTranslation2d()
			.rotateBy(Rotation2d.kCCW_90deg)
			.times(gyroYawAngularVelocity.getRadians())
			.rotateBy(robotPose.getRotation());
		Translation2d turretFieldRelativeVelocity = robotTranslationalVelocity.plus(turretTangentialVelocity);

		Translation2d turretPredictedPose = getPredictedTurretPose(
			fieldRelativeTurretTranslation,
			turretFieldRelativeVelocity,
			distanceFromTurretToTargetMeters
		);

		Rotation2d predictedAngleToTarget = targetTranslation.minus(turretPredictedPose).getAngle();

		// Turret FeedForward
		Translation2d targetRelativeTurretVelocity = turretFieldRelativeVelocity.rotateBy(predictedAngleToTarget.unaryMinus());
		Rotation2d targetTurretVelocityCausedByTranslation = Rotation2d
			.fromRadians(-targetRelativeTurretVelocity.getY() / distanceFromTurretToTargetMeters);
		Rotation2d turretTargetVelocityRPS = Rotation2d
			.fromRadians(targetTurretVelocityCausedByTranslation.getRadians() - gyroYawAngularVelocity.getRadians());

		double distanceFromTurretPredictedPoseToHub = getDistanceFromHub(turretPredictedPose);
		Rotation2d turretTargetPosition = predictedAngleToTarget.minus(robotPose.getRotation());
		Rotation2d hoodTargetPosition = hoodInterpolation.get(distanceFromTurretPredictedPoseToHub);
		Rotation2d flywheelTargetRPS = flywheelInterpolation.get(distanceFromTurretPredictedPoseToHub);

		Logger.recordOutput(LOG_PATH + "/turretFieldRelativePose", new Pose2d(fieldRelativeTurretTranslation, new Rotation2d()));
		Logger.recordOutput(LOG_PATH + "/turretTarget", turretTargetPosition);
		Logger.recordOutput(LOG_PATH + "/turretTargetVelocityRPS", turretTargetVelocityRPS);
		Logger.recordOutput(LOG_PATH + "/hoodTarget", hoodTargetPosition);
		Logger.recordOutput(LOG_PATH + "/flywheelTarget", flywheelTargetRPS);
		Logger.recordOutput(LOG_PATH + "/predictedTurretPose", new Pose2d(turretPredictedPose, new Rotation2d()));
		Logger.recordOutput(LOG_PATH + "/distanceFromTarget", distanceFromTurretToTargetMeters);
		return new ShootingParams(
			flywheelTargetRPS,
			hoodTargetPosition,
			turretTargetPosition,
			turretTargetVelocityRPS,
			turretPredictedPose,
			targetTranslation
		);
	}

	private static ShootingParams calculateScoringParams(
		Pose2d robotPose,
		ChassisSpeeds fieldRelativeSpeeds,
		Rotation2d gyroYawAngularVelocity
	) {
		return calculateShootingParams(
			robotPose,
			fieldRelativeSpeeds,
			gyroYawAngularVelocity,
			ShootingInterpolations.HOOD_SCORING_INTERPOLATION_MAP,
			ShootingInterpolations.FLYWHEEL_SCORING_INTERPOLATION_MAP,
			Field.getHubMiddle()
		);
	}

	private static ShootingParams calculatePassingParams(
		Pose2d robotPose,
		ChassisSpeeds fieldRelativeSpeeds,
		Rotation2d gyroYawAngularVelocity
	) {
		return calculateShootingParams(
			robotPose,
			fieldRelativeSpeeds,
			gyroYawAngularVelocity,
			ShootingInterpolations.HOOD_PASSING_INTERPOLATION_MAP,
			ShootingInterpolations.FLYWHEEL_PASSING_INTERPOLATION_MAP,
			getOptimalPassingPosition(robotPose)
		);
	}

	private static Translation2d getPredictedTurretPose(Translation2d turretPose, Translation2d turretVelocities, double distanceFromHubMeters) {
		double ballFlightTime = ShootingInterpolations.DISTANCE_TO_BALL_FLIGHT_TIME_INTERPOLATION_MAP.get(distanceFromHubMeters);

		double turretPosePredictionX = turretPose.getX() + (turretVelocities.getX() * ballFlightTime);
		double turretPosePredictionY = turretPose.getY() + (turretVelocities.getY() * ballFlightTime);

		return new Translation2d(turretPosePredictionX, turretPosePredictionY);
	}

	public static Translation2d getFieldRelativeTurretPosition(Pose2d robotPose) {
		Translation2d turretPositionRelativeToRobotRelativeToField = TurretConstants.TURRET_POSITION_RELATIVE_TO_ROBOT.toTranslation2d()
			.rotateBy(robotPose.getRotation());
		return new Translation2d(
			robotPose.getX() + turretPositionRelativeToRobotRelativeToField.getX(),
			robotPose.getY() + turretPositionRelativeToRobotRelativeToField.getY()
		);
	}

	public static double getDistanceFromHub(Translation2d pose) {
		return Field.getHubMiddle().getDistance(pose);
	}

	public static Translation2d getOptimalPassingPosition(Pose2d robotPose) {
		// to do
		return new Translation2d(3, 4);
	}

	public static void updateShootingParams(Pose2d robotPose, ChassisSpeeds speedsFieldRelative, Rotation2d gyroYawAngularVelocity) {
		if (ShootingChecks.isInAllianceZone(robotPose.getTranslation())) {
			shootingParams = calculateScoringParams(robotPose, speedsFieldRelative, gyroYawAngularVelocity);
		} else {
			shootingParams = calculatePassingParams(robotPose, speedsFieldRelative, gyroYawAngularVelocity);
		}
	}

}
