package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.utils.LoggedNetworkRotation2d;
import frc.utils.math.FieldMath;

public class ShooterConstants {

	public static final Rotation2d DEFAULT_FLYWHEEL_ROTATIONS_PER_SECOND = Rotation2d.fromRotations(10);

	public static final double MAX_Y_VALUE_FOR_UNPASSABLE_AREA = Field.getHubMiddle().getY() + Field.HUB_Y_AXIS_LENGTH_METERS / 2;
	public static final double MIN_Y_VALUE_FOR_UNPASSABLE_AREA = Field.getHubMiddle().getY() - Field.HUB_Y_AXIS_LENGTH_METERS / 2;

	private static final double MAX_X_VALUE_FOR_UNPASSABLE_AREA = 8;
	private static final double TARGET_X_VALUE_FOR_PASSING = 3;

	private static final Translation2d LOWER_Y_SIDE_PASSING_TARGET = new Translation2d(1, 1);
	private static final Translation2d UPPER_Y_SIDE_PASSING_TARGET = FieldMath.mirror(LOWER_Y_SIDE_PASSING_TARGET, false, true);

	public static double getTargetXForPassing() {
		if (!Field.isFieldConventionAlliance()) {
			return FieldMath.mirrorX(TARGET_X_VALUE_FOR_PASSING);
		}
		return TARGET_X_VALUE_FOR_PASSING;
	}

	public static Translation2d getLowerYSidePassingTarget() {
		if (!Field.isFieldConventionAlliance()) {
			return FieldMath.mirror(LOWER_Y_SIDE_PASSING_TARGET, true, false);
		}
		return LOWER_Y_SIDE_PASSING_TARGET;
	}

	public static Translation2d getUpperYSidePassingTarget() {
		if (!Field.isFieldConventionAlliance()) {
			return FieldMath.mirror(UPPER_Y_SIDE_PASSING_TARGET, true, false);
		}
		return UPPER_Y_SIDE_PASSING_TARGET;
	}

	public static double getMaxXValueForUnpassableArea() {
		if (Field.isFieldConventionAlliance()) {
			return MAX_X_VALUE_FOR_UNPASSABLE_AREA;
		}
		return FieldMath.mirrorX(MAX_X_VALUE_FOR_UNPASSABLE_AREA);
	}

	public static final LoggedNetworkRotation2d turretCalibrationAngle = new LoggedNetworkRotation2d(
		"Tunable/TurretAngle",
		Rotation2d.fromDegrees(0.0)
	);
	public static final LoggedNetworkRotation2d hoodCalibrationAngle = new LoggedNetworkRotation2d(
		"Tunable/HoodAngle",
		Rotation2d.fromDegrees(0.0)
	);
	public static final LoggedNetworkRotation2d flywheelCalibrationRotations = new LoggedNetworkRotation2d(
		"Tunable/FlywheelRotations",
		Rotation2d.fromRotations(0.0)
	);

}
