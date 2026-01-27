package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.constants.field.Field;
import frc.robot.statemachine.shooterstatehandler.ShootingParams;
import frc.robot.subsystems.constants.hood.HoodConstants;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.utils.InterpolationMap;
import frc.utils.driverstation.DriverStationUtil;
import frc.utils.math.FieldMath;
import org.littletonrobotics.junction.Logger;

import java.util.Map;

public class ShootingCalculations {

	private static final String LOG_PATH = "ShootingCalculations";
	private static ShootingParams shootingParams = new ShootingParams(
		new Rotation2d(),
		HoodConstants.MINIMUM_POSITION,
		TurretConstants.MIN_POSITION,
		new Rotation2d()
	);

	public static ShootingParams getShootingParams() {
		return shootingParams;
	}

	private static ShootingParams calculateShootingParams(Pose2d robotPose) {
		Rotation2d turretTargetPosition = getRobotRelativeLookAtHubAngleForTurret(robotPose);

		double distanceFromHubMeters = getDistanceFromHub(robotPose.getTranslation());
		Rotation2d hoodTargetPosition = hoodHubShootingInterpolation(distanceFromHubMeters);
		Rotation2d flywheelTargetRPS = flywheelHubShootingInterpolation(distanceFromHubMeters);

		Logger.recordOutput(LOG_PATH + "/turretTarget", turretTargetPosition);
		Logger.recordOutput(LOG_PATH + "/hoodTarget", hoodTargetPosition);
		Logger.recordOutput(LOG_PATH + "/flywheelTarget", flywheelTargetRPS);
		return new ShootingParams(flywheelTargetRPS, hoodTargetPosition, turretTargetPosition, new Rotation2d());
	}

	private static ShootingParams calculatePassingParams(Pose2d robotPose,Translation2d passingTarget) {
		Rotation2d turretTargetPosition = getRobotRelativeLookAtHubAngleForTurret(robotPose);

		double distanceFromTargetMeters = passingTarget.getDistance(robotPose.getTranslation());
		Rotation2d hoodTargetPosition = hoodPassingInterpolation(distanceFromTargetMeters);
		Rotation2d flywheelTargetRPS = flywheelPassingInterpolation(distanceFromTargetMeters);

		Logger.recordOutput(LOG_PATH + "/turretTarget", turretTargetPosition);
		Logger.recordOutput(LOG_PATH + "/hoodTarget", hoodTargetPosition);
		Logger.recordOutput(LOG_PATH + "/flywheelTarget", flywheelTargetRPS);
		return new ShootingParams(flywheelTargetRPS, hoodTargetPosition, turretTargetPosition, new Rotation2d());
	}

	private static Translation2d getFieldRelativeTurretPosition(Pose2d robotPose) {
		Translation2d turretPositionRelativeToRobotRelativeToField = TurretConstants.TURRET_POSITION_RELATIVE_TO_ROBOT.toTranslation2d()
			.rotateBy(robotPose.getRotation());
		return new Translation2d(
			robotPose.getX() + turretPositionRelativeToRobotRelativeToField.getX(),
			robotPose.getY() + turretPositionRelativeToRobotRelativeToField.getY()
		);
	}

	private static Rotation2d getRobotRelativeLookAtHubAngleForTurret(Pose2d robotPose) {
		Translation2d fieldRelativeTurretPose = getFieldRelativeTurretPosition(robotPose);
		return FieldMath.getRelativeTranslation(fieldRelativeTurretPose, Field.getHubMiddle()).getAngle().minus(robotPose.getRotation());
	}

	public static double getDistanceFromHub(Translation2d pose) {
		return Field.getHubMiddle().getDistance(pose);
	}

	public static Translation2d getOptimalPassingPosition(Translation2d robotPosition) {
		return new Translation2d(3, 3);
	}

	private static final InterpolationMap<Double, Rotation2d> HOOD_INTERPOLATION_MAP_SHOOT_AT_HUB = new InterpolationMap<Double, Rotation2d>(
		InverseInterpolator.forDouble(),
		InterpolationMap.interpolatorForRotation2d(),
		Map.of(
			1.5,
			Rotation2d.fromDegrees(67),
			3.0,
			Rotation2d.fromDegrees(60),
			4.5,
			Rotation2d.fromDegrees(43),
			6.0,
			Rotation2d.fromDegrees(33)
		)
	);

	private static final InterpolationMap<Double, Rotation2d> FLYWHEEL_INTERPOLATION_MAP_SHOOT_AT_HUB = new InterpolationMap<Double, Rotation2d>(
		InverseInterpolator.forDouble(),
		InterpolationMap.interpolatorForRotation2d(),
		Map.of(
			1.5,
			Rotation2d.fromRotations(50),
			3.0,
			Rotation2d.fromRotations(65),
			4.5,
			Rotation2d.fromRotations(85),
			6.0,
			Rotation2d.fromRotations(110)
		)
	);

	private static final InterpolationMap<Double, Rotation2d> HOOD_INTERPOLATION_MAP_PASS = new InterpolationMap<Double, Rotation2d>(
		InverseInterpolator.forDouble(),
		InterpolationMap.interpolatorForRotation2d(),
		Map.of(
			1.5,
			Rotation2d.fromDegrees(67),
			3.0,
			Rotation2d.fromDegrees(60),
			4.5,
			Rotation2d.fromDegrees(43),
			6.0,
			Rotation2d.fromDegrees(33)
		)
	);

	private static final InterpolationMap<Double, Rotation2d> FLYWHEEL_INTERPOLATION_MAP_PASS = new InterpolationMap<Double, Rotation2d>(
		InverseInterpolator.forDouble(),
		InterpolationMap.interpolatorForRotation2d(),
		Map.of(
			1.5,
			Rotation2d.fromRotations(50),
			3.0,
			Rotation2d.fromRotations(65),
			4.5,
			Rotation2d.fromRotations(85),
			6.0,
			Rotation2d.fromRotations(110)
		)
	);

	public static Rotation2d hoodHubShootingInterpolation(double distanceFromTower) {
		return HOOD_INTERPOLATION_MAP_SHOOT_AT_HUB.get(distanceFromTower);
	}

	public static Rotation2d flywheelHubShootingInterpolation(double distanceFromTower) {
		return FLYWHEEL_INTERPOLATION_MAP_SHOOT_AT_HUB.get(distanceFromTower);
	}

	public static Rotation2d hoodPassingInterpolation(double distanceFromTower) {
		return HOOD_INTERPOLATION_MAP_PASS.get(distanceFromTower);
	}

	public static Rotation2d flywheelPassingInterpolation(double distanceFromTower) {
		return FLYWHEEL_INTERPOLATION_MAP_PASS.get(distanceFromTower);
	}


	public static void updateShootingParams(Pose2d robotPose) {
		if (ShootingChecks.isInAllianceZone(robotPose.getTranslation())) {
			shootingParams = calculateShootingParams(robotPose);
		} else {
			shootingParams = calculatePassingParams(robotPose, getOptimalPassingPosition(robotPose.getTranslation()));
		}
	}

}
