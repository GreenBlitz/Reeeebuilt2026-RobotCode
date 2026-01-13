package frc.robot.statemachine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.constants.MathConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.utils.InterpolationMap;
import frc.utils.math.FieldMath;
import frc.utils.math.ToleranceMath;

import java.util.Map;

public class ShooterCalculations {

	public static Translation3d getFieldRelativeTurretTranslation3d(Pose2d robotPose) {
		Translation2d turretPositionRelativeToRobotRelativeToField = TurretConstants.TURRET_POSITION_RELATIVE_TO_ROBOT.toTranslation2d()
			.rotateBy(robotPose.getRotation());
		return new Translation3d(
			robotPose.getX() + turretPositionRelativeToRobotRelativeToField.getX(),
			robotPose.getY() + turretPositionRelativeToRobotRelativeToField.getY(),
			TurretConstants.TURRET_POSITION_RELATIVE_TO_ROBOT.getZ()
		);
	}

	public static Pose2d getFieldRelativeTurretPosition(Pose2d robotPose) {
		Translation2d turretPositionRelativeToRobotRelativeToField = TurretConstants.TURRET_POSITION_RELATIVE_TO_ROBOT.toTranslation2d()
			.rotateBy(robotPose.getRotation());
		return new Pose2d(
			new Translation2d(
				robotPose.getX() + turretPositionRelativeToRobotRelativeToField.getX(),
				robotPose.getY() + turretPositionRelativeToRobotRelativeToField.getY()
			),
			robotPose.getRotation()
		);
	}

	public static boolean isTurretMoveLegal(Rotation2d targetRobotRelative, Arm turret) {
		boolean isTargetInMaxRange = !(targetRobotRelative.getDegrees() > TurretConstants.SCREW_MAX_RANGE_EDGE.getDegrees()
			&& turret.getPosition().getDegrees() < TurretConstants.SCREW_MIN_RANGE_EDGE.getDegrees());

		boolean isTargetInMinRange = !(targetRobotRelative.getDegrees() < TurretConstants.SCREW_MIN_RANGE_EDGE.getDegrees()
			&& turret.getPosition().getDegrees() > TurretConstants.SCREW_MAX_RANGE_EDGE.getDegrees());

		boolean isTargetBehindSoftwareLimits = ToleranceMath.isInRange(
			targetRobotRelative.getDegrees(),
			TurretConstants.BACKWARDS_SOFTWARE_LIMIT.getDegrees(),
			TurretConstants.FORWARD_SOFTWARE_LIMIT.getDegrees()
		);

		return isTargetInMaxRange && isTargetInMinRange && isTargetBehindSoftwareLimits;
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

	public static Rotation2d getRobotRelativeLookAtHubAngleForTurret(Translation2d target, Pose2d fieldRelativeTurretPose) {
		Rotation2d targetAngle = Rotation2d
			.fromRadians(FieldMath.getRelativeTranslation(fieldRelativeTurretPose, target).getAngle().getRadians());
		return Rotation2d
			.fromDegrees(MathUtil.inputModulus(targetAngle.getDegrees(), Rotation2d.kZero.getDegrees(), MathConstants.FULL_CIRCLE.getDegrees()));
	}

	public static Rotation2d getRangeEdge(Rotation2d angle, Rotation2d tolerance) {
		return Rotation2d.fromRadians(
			MathUtil
				.inputModulus(angle.getRadians() + tolerance.getRadians(), Rotation2d.kZero.getRadians(), MathConstants.FULL_CIRCLE.getRadians())
		);
	}

	public static Rotation2d hoodInterpolation(double distanceFromTower) {
		return HOOD_INTERPOLATION_MAP.get(distanceFromTower);
	}

	public static Rotation2d flywheelInterpolation(double distanceFromTower) {
		return FLYWHEEL_INTERPOLATION_MAP.get(distanceFromTower);
	}

}
