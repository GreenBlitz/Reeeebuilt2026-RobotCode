package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.field.Field;
import frc.utils.LoggedNetworkRotation2d;

public class ShooterConstants {

	public static final Rotation2d DEFAULT_FLYWHEEL_ROTATIONS_PER_SECOND = Rotation2d.fromRotations(10);

	public static final double TIME_TO_CLOSE_HOOD_WITH_BUFFER_SEC = 0.5;
	public static final double DISTANCH_FROM_TRENCH_CENTER_TO_CLOSE_HOOD = Field.TRENCH_BAR_X_AXIS_LENGTH_METERS * 2;

	public static final Rotation2d MIN_HOOD_POSITION_TO_GO_UNDER_TRENCH = Rotation2d.fromDegrees(50);

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
