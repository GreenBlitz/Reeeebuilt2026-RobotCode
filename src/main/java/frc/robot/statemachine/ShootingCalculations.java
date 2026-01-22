package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.field.Field;
import frc.robot.statemachine.shooterstatehandler.ShootingParams;
import frc.robot.subsystems.constants.hood.HoodConstants;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.utils.InterpolationMap;
import org.littletonrobotics.junction.Logger;

import java.util.Map;

public class ShootingCalculations {

	private static final String LOG_PATH = "ShootingCalculations";
	private static ShootingParams shootingParams = new ShootingParams(
		new Rotation2d(),
		HoodConstants.MINIMUM_POSITION,
		TurretConstants.MIN_POSITION,
		new Rotation2d(),
		new Translation2d()
	);

	public static ShootingParams getShootingParams() {
		return shootingParams;
	}

	private static ShootingParams calculateShootingParams(
		Pose2d robotPose,
		ChassisSpeeds fieldRelativeSpeeds,
		Rotation2d gyroYawAngularVelocity
	) {
		// Calculate distance from turret to target
		Translation2d hubTranslation = Field.getHubMiddle();
		Translation2d fieldRelativeTurretTranslation = getFieldRelativeTurretPosition(robotPose);
		double turretToHubDistanceMeters = hubTranslation.getDistance(fieldRelativeTurretTranslation);
		Logger.recordOutput(LOG_PATH+"aagsdjanfa",turretToHubDistanceMeters);
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
			turretToHubDistanceMeters
		);

		Rotation2d angleToTarget = hubTranslation.minus(turretPredictedPose).getAngle();

		// Turret FeedForward
		Translation2d targetRelativeTurretVelocity = turretFieldRelativeVelocity.rotateBy(angleToTarget.unaryMinus());
		Rotation2d targetTurretVelocityCausedByTranslation = Rotation2d
			.fromRadians(-targetRelativeTurretVelocity.getY() / turretToHubDistanceMeters);
		Rotation2d turretTargetVelocityRPS = Rotation2d
			.fromRadians(targetTurretVelocityCausedByTranslation.getRadians() - gyroYawAngularVelocity.getRadians());

		double distanceFromTurretPredictedPoseToHub = getDistanceFromHub(turretPredictedPose);
		Rotation2d turretTargetPosition = angleToTarget.minus(robotPose.getRotation());
		Rotation2d hoodTargetPosition = hoodInterpolation(distanceFromTurretPredictedPoseToHub);
		Rotation2d flywheelTargetRPS = flywheelInterpolation(distanceFromTurretPredictedPoseToHub);
		Logger.recordOutput(LOG_PATH + "/turretOnPose", new Pose2d(fieldRelativeTurretTranslation,new Rotation2d()));
		Logger.recordOutput(LOG_PATH + "/turretTarget", turretTargetPosition);
		Logger.recordOutput(LOG_PATH + "/turretTargetVelocityRPS", turretTargetVelocityRPS);
		Logger.recordOutput(LOG_PATH + "/hoodTarget", hoodTargetPosition);
		Logger.recordOutput(LOG_PATH + "/flywheelTarget", flywheelTargetRPS);
		Logger.recordOutput(LOG_PATH + "/predictedTurretPose", new Pose2d(turretPredictedPose, new Rotation2d()));
		return new ShootingParams(flywheelTargetRPS, hoodTargetPosition, turretTargetPosition, turretTargetVelocityRPS, turretPredictedPose);
	}

	private static Translation2d getFieldRelativeTurretPosition(Pose2d robotPose) {
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

	private static final InterpolationMap<Double, Rotation2d> HOOD_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
		InverseInterpolator.forDouble(),
		InterpolationMap.interpolatorForRotation2d(),
		Map.of(
			2.0,
			Rotation2d.fromDegrees(60),
			2.5,
			Rotation2d.fromDegrees(60),
			3.0,
			Rotation2d.fromDegrees(60),
			3.5,
			Rotation2d.fromDegrees(45),
			4.0,
			Rotation2d.fromDegrees(40),
			4.5,
			Rotation2d.fromDegrees(40),
			5.0,
			Rotation2d.fromDegrees(40),
			5.5,
			Rotation2d.fromDegrees(44)
		)
	);

	private static final InterpolationMap<Double, Rotation2d> FLYWHEEL_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
		InverseInterpolator.forDouble(),
		InterpolationMap.interpolatorForRotation2d(),
		Map.of(
			2.0,
			Rotation2d.fromDegrees(16500),
			2.5,
			Rotation2d.fromDegrees(17500),
			3.0,
			Rotation2d.fromDegrees(20500),
			3.5,
			Rotation2d.fromDegrees(20000),
			4.0,
			Rotation2d.fromDegrees(22000),
			4.5,
			Rotation2d.fromDegrees(23000),
			5.0,
			Rotation2d.fromDegrees(24500),
			5.5,
			Rotation2d.fromDegrees(25500)
		)
	);

	public static Rotation2d hoodInterpolation(double distanceFromTower) {
		return HOOD_INTERPOLATION_MAP.get(distanceFromTower);
	}

	public static Rotation2d flywheelInterpolation(double distanceFromTower) {
		return FLYWHEEL_INTERPOLATION_MAP.get(distanceFromTower);
	}

	private static final InterpolationMap<Double, Double> DISTANCE_TO_BALL_FLIGHT_TIME_INTERPOLATION_MAP = new InterpolationMap<Double, Double>(
		InverseInterpolator.forDouble(),
		Interpolator.forDouble(),
		Map.of(
				2.603,
				1.0,
				3.3,
				1.1,
				4.0,
				1.6)
	);

	private static Translation2d getPredictedTurretPose(Translation2d turretPose, Translation2d turretVelocities, double distanceFromHubMeters) {
		double ballFlightTime = DISTANCE_TO_BALL_FLIGHT_TIME_INTERPOLATION_MAP.get(distanceFromHubMeters);

		double turretPosePredictionX = turretPose.getX() + (turretVelocities.getX() * ballFlightTime);
		double turretPosePredictionY = turretPose.getY() + (turretVelocities.getY() * ballFlightTime);

		return new Translation2d(turretPosePredictionX, turretPosePredictionY);
	}

	public static void updateShootingParams(Pose2d robotPose, ChassisSpeeds speedsFieldRelative, Rotation2d gyroYawAngularVelocity) {
		shootingParams = calculateShootingParams(robotPose, speedsFieldRelative, gyroYawAngularVelocity);
	}

}
