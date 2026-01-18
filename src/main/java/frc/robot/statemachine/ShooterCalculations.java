package frc.robot.statemachine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.field.Field;
import frc.robot.statemachine.shooterstatehandler.ShootingParams;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.utils.InterpolationMap;
import frc.utils.math.FieldMath;
import frc.utils.math.ToleranceMath;
import org.littletonrobotics.junction.Logger;

import java.util.Map;

public class ShooterCalculations {

	private static final String LOG_PATH = "ShooterCalculations";

	public static ShootingParams getShootingParams(Pose2d robotPose, ChassisSpeeds speedsFieldRelative, Rotation2d turretPosition) {
		Translation2d hubTranslation = Field.getHubMiddle();
		Translation2d robotTranslationalVel = new Translation2d(speedsFieldRelative.vxMetersPerSecond, speedsFieldRelative.vyMetersPerSecond);
		Rotation2d robotAngularVel = Rotation2d.fromRadians(speedsFieldRelative.omegaRadiansPerSecond);

		Translation2d turretFieldRelativeTranslation = getFieldRelativeTurretPosition(robotPose, turretPosition).getTranslation();

		Translation2d turretTangentialVel = new Translation2d(
			-TurretConstants.TURRET_POSITION_RELATIVE_TO_ROBOT.getY(),
			TurretConstants.TURRET_POSITION_RELATIVE_TO_ROBOT.getX()
		).times(robotAngularVel.getRadians());
		Translation2d totalTurretFieldVel = robotTranslationalVel.plus(turretTangentialVel);

		Rotation2d angleToTarget = hubTranslation.minus(turretFieldRelativeTranslation).getAngle();
		Translation2d targetRelativeTurretVel = totalTurretFieldVel.rotateBy(angleToTarget.unaryMinus());

		double perpendicularVel = targetRelativeTurretVel.getY();
		double distanceFromHubMeters = hubTranslation.getDistance(turretFieldRelativeTranslation);

		Rotation2d turretFieldAngularVelocity = Rotation2d.fromRadians(-perpendicularVel / distanceFromHubMeters);
		Rotation2d turretFeedforward = turretFieldAngularVelocity.minus(robotAngularVel);

		Rotation2d turretTargetPosition = wrapTurretPosition(angleToTarget.minus(robotPose.getRotation()));
		Rotation2d hoodTargetPosition = hoodInterpolation(distanceFromHubMeters);
		Rotation2d flywheelTargetRPS = flywheelInterpolation(distanceFromHubMeters);

		Logger.recordOutput(LOG_PATH + "/turretTarget", turretTargetPosition);
		Logger.recordOutput(LOG_PATH + "/turretFF", turretFeedforward);
		Logger.recordOutput(LOG_PATH + "/hoodTarget", hoodTargetPosition);
		Logger.recordOutput(LOG_PATH + "/flywheelTarget", flywheelTargetRPS);
		return new ShootingParams(flywheelTargetRPS, hoodTargetPosition, turretTargetPosition, turretFeedforward);
	}

	public static double getDistanceFromHub(Translation2d pose) {
		return Field.getHubMiddle().getDistance(pose);
	}

	public static Pose2d getFieldRelativeTurretPosition(Pose2d robotPose, Rotation2d turretAngle) {
		Translation2d turretPositionRelativeToRobotRelativeToField = TurretConstants.TURRET_POSITION_RELATIVE_TO_ROBOT.toTranslation2d()
			.rotateBy(robotPose.getRotation());
		return new Pose2d(
			new Translation2d(
				robotPose.getX() + turretPositionRelativeToRobotRelativeToField.getX(),
				robotPose.getY() + turretPositionRelativeToRobotRelativeToField.getY()
			),
			Rotation2d.fromDegrees(robotPose.getRotation().getDegrees() + turretAngle.getDegrees())
		);
	}

	public static boolean isTurretMoveLegal(Rotation2d targetRobotRelative, Rotation2d position) {
		boolean isTargetInMaxRange = !(targetRobotRelative.getDegrees() > TurretConstants.SCREW_MAX_RANGE_EDGE.getDegrees()
			&& position.getDegrees() < TurretConstants.SCREW_MIN_RANGE_EDGE.getDegrees());

		boolean isTargetInMinRange = !(targetRobotRelative.getDegrees() < TurretConstants.SCREW_MIN_RANGE_EDGE.getDegrees()
			&& position.getDegrees() > TurretConstants.SCREW_MAX_RANGE_EDGE.getDegrees());

		boolean isTargetBehindSoftwareLimits = ToleranceMath.isInRange(
			targetRobotRelative.getDegrees(),
			TurretConstants.BACKWARDS_SOFTWARE_LIMIT.getDegrees(),
			TurretConstants.FORWARD_SOFTWARE_LIMIT.getDegrees()
		);

		return isTargetInMaxRange && isTargetInMinRange && isTargetBehindSoftwareLimits;
	}

	public static Rotation2d getRobotRelativeLookAtHubAngleForTurret(Pose2d robotPose, Rotation2d turretPosition) {
		Translation2d fieldRelativeTurretPose = getFieldRelativeTurretPosition(robotPose, turretPosition).getTranslation();
		Rotation2d targetAngle = Rotation2d.fromDegrees(
			FieldMath.getRelativeTranslation(fieldRelativeTurretPose, Field.getHubMiddle()).getAngle().getDegrees()
				- robotPose.getRotation().getDegrees()
		);
		return wrapTurretPosition(targetAngle);
	}

	public static Rotation2d wrapTurretPosition(Rotation2d turretPosition) {
		return Rotation2d.fromRadians(
			MathUtil
				.inputModulus(turretPosition.getRadians(), TurretConstants.MIN_POSITION.getRadians(), TurretConstants.MAX_POSITION.getRadians())
		);
	}

	public static Rotation2d getRangeEdge(Rotation2d angle, Rotation2d tolerance) {
		return Rotation2d.fromRadians(
			MathUtil.inputModulus(
				angle.getRadians() + tolerance.getRadians(),
				TurretConstants.MIN_POSITION.getRadians(),
				TurretConstants.MAX_POSITION.getRadians()
			)
		);
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

}
