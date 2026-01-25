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
		Rotation2d hoodTargetPosition = hoodInterpolation(distanceFromHubMeters);
		Rotation2d flywheelTargetRPS = flywheelInterpolation(distanceFromHubMeters);

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

	private static final InterpolationMap<Double, Rotation2d> HOOD_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
		InverseInterpolator.forDouble(),
		InterpolationMap.interpolatorForRotation2d(),
		Map.of(
			0.8,
			Rotation2d.fromDegrees(67),
			1.5,
			Rotation2d.fromDegrees(60),
			2.5,
			Rotation2d.fromDegrees(43),
			3.7,
			Rotation2d.fromDegrees(33)
		)
	);

	private static final InterpolationMap<Double, Rotation2d> FLYWHEEL_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
		InverseInterpolator.forDouble(),
		InterpolationMap.interpolatorForRotation2d(),
		Map.of(
			0.8,
			Rotation2d.fromDegrees(7000),
			1.5,
			Rotation2d.fromDegrees(7800),
			2.5,
			Rotation2d.fromDegrees(10000),
			3.7,
			Rotation2d.fromDegrees(12000)
		)
	);

	public static Rotation2d hoodInterpolation(double distanceFromTower) {
		return HOOD_INTERPOLATION_MAP.get(distanceFromTower);
	}

	public static Rotation2d flywheelInterpolation(double distanceFromTower) {
		return FLYWHEEL_INTERPOLATION_MAP.get(distanceFromTower);
	}

	public static void updateShootingParams(Pose2d robotPose) {
		shootingParams = calculateShootingParams(robotPose);
	}

}
