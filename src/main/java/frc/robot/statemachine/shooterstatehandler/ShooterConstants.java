package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.utils.LoggedNetworkRotation2d;
import frc.utils.math.FieldMath;

public class ShooterConstants {

	public static final Rotation2d DEFAULT_FLYWHEEL_ROTATIONS_PER_SECOND = Rotation2d.fromRotations(10);

	public static final double MAX_Y_FOR_UNPASSABLE_AREA = Field.getHubMiddle().getY() + 0.58;
	public static final double MIN_Y_FOR_UNPASSABLE_AREA = Field.getHubMiddle().getY() - 0.58;

	public static final double MAX_X_FOR_UNPASSABLE_ARIA_BLUE_RELATIVE = 8;
	private static final double TARGET_X_FOR_PASSING_BLUE_RELATIVE = 3;

	private static final Translation2d LOWER_Y_SIDE_PASSING_TARGET = new Translation2d(2, 2);
	private static final Translation2d UPPER_Y_SIDE_PASSING_TARGET = FieldMath.mirror(LOWER_Y_SIDE_PASSING_TARGET, false, true);

	public static final double getTargetXForPassing() {
		if (!Field.isFieldConventionAlliance()) {
			return FieldMath.mirrorX(TARGET_X_FOR_PASSING_BLUE_RELATIVE);
		}
		return TARGET_X_FOR_PASSING_BLUE_RELATIVE;
	}

	public static final Translation2d getLowerYSidePassingTarget() {
		if (!Field.isFieldConventionAlliance()) {
			return FieldMath.mirror(LOWER_Y_SIDE_PASSING_TARGET, true, false);
		}
		return LOWER_Y_SIDE_PASSING_TARGET;
	}

	public static final Translation2d getUpperYSidePassingTarget() {
		if (!Field.isFieldConventionAlliance()) {
			return FieldMath.mirror(UPPER_Y_SIDE_PASSING_TARGET, true, false);
		}
		return UPPER_Y_SIDE_PASSING_TARGET;
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
