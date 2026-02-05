package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.utils.InterpolationMap;

import java.util.Map;

public class ShootingInterpolations {

	public static final InterpolationMap<Double, Rotation2d> HOOD_SCORING_INTERPOLATION_MAP;

	public static final InterpolationMap<Double, Rotation2d> FLYWHEEL_SCORING_INTERPOLATION_MAP;

	public static final InterpolationMap<Double, Rotation2d> HOOD_PASSING_INTERPOLATION_MAP;

	public static final InterpolationMap<Double, Rotation2d> FLYWHEEL_PASSING_INTERPOLATION_MAP;

	public static final InterpolationMap<Double, Double> DISTANCE_TO_BALL_FLIGHT_TIME_INTERPOLATION_MAP;

	public static final InterpolationMap<Double, Rotation2d> FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_START_SCORING_INTERPOLATION_MAP;

	public static final InterpolationMap<Double, Rotation2d> FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_CONTINUE_SCORING_INTERPOLATION_MAP;

	public static final InterpolationMap<Double, Rotation2d> HOOD_POSITION_TOLERANCE_TO_START_SCORING_INTERPOLATION_MAP;

	public static final InterpolationMap<Double, Rotation2d> HOOD_POSITION_TOLERANCE_TO_CONTINUE_SCORING_INTERPOLATION_MAP;

	static {
		HOOD_SCORING_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
			InverseInterpolator.forDouble(),
			InterpolationMap.interpolatorForRotation2d(),
			Map.of(
				1.5,
				Rotation2d.fromDegrees(30),
				2.0,
				Rotation2d.fromDegrees(30),
				2.5,
				Rotation2d.fromDegrees(30),
				3.0,
				Rotation2d.fromDegrees(35),
				3.6,
				Rotation2d.fromDegrees(37),
				4.0,
				Rotation2d.fromDegrees(38),
				4.5,
				Rotation2d.fromDegrees(43),
				5.0,
				Rotation2d.fromDegrees(40),
				5.5,
				Rotation2d.fromDegrees(45),
				6.0,
				Rotation2d.fromDegrees(44)
			)
		);

		FLYWHEEL_SCORING_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
			InverseInterpolator.forDouble(),
			InterpolationMap.interpolatorForRotation2d(),
			Map.of(
				1.5,
				Rotation2d.fromDegrees(16400),
				2.0,
				Rotation2d.fromDegrees(17200),
				2.55,
				Rotation2d.fromDegrees(19300),
				3.0,
				Rotation2d.fromDegrees(20400),
				3.45,
				Rotation2d.fromDegrees(20800),
				4.04,
				Rotation2d.fromDegrees(21800),
				4.48,
				Rotation2d.fromDegrees(23000),
				5.0,
				Rotation2d.fromDegrees(23350),
				5.5,
				Rotation2d.fromDegrees(24350),
				6.0,
				Rotation2d.fromDegrees(25350)
			)
		);

		HOOD_PASSING_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
			InverseInterpolator.forDouble(),
			InterpolationMap.interpolatorForRotation2d(),
			Map.of(
				1.5,
				Rotation2d.fromDegrees(30),
				2.0,
				Rotation2d.fromDegrees(30),
				2.5,
				Rotation2d.fromDegrees(30),
				3.0,
				Rotation2d.fromDegrees(35),
				3.6,
				Rotation2d.fromDegrees(37),
				4.0,
				Rotation2d.fromDegrees(38),
				4.5,
				Rotation2d.fromDegrees(43),
				5.0,
				Rotation2d.fromDegrees(40),
				5.5,
				Rotation2d.fromDegrees(45),
				6.0,
				Rotation2d.fromDegrees(44)
			)
		);

		FLYWHEEL_PASSING_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
			InverseInterpolator.forDouble(),
			InterpolationMap.interpolatorForRotation2d(),
			Map.of(
				1.5,
				Rotation2d.fromDegrees(16400),
				2.0,
				Rotation2d.fromDegrees(16500),
				2.5,
				Rotation2d.fromDegrees(18000),
				3.0,
				Rotation2d.fromDegrees(18500),
				3.6,
				Rotation2d.fromDegrees(19700),
				4.0,
				Rotation2d.fromDegrees(20800),
				4.5,
				Rotation2d.fromDegrees(21500),
				5.0,
				Rotation2d.fromDegrees(23000),
				5.5,
				Rotation2d.fromDegrees(23700),
				6.0,
				Rotation2d.fromDegrees(24800)
			)
		);

		DISTANCE_TO_BALL_FLIGHT_TIME_INTERPOLATION_MAP = new InterpolationMap<Double, Double>(
			InverseInterpolator.forDouble(),
			Interpolator.forDouble(),
			Map.of(2.0, 0.9, 4.0, 1.16, 5.85, 1.31)
		);

		FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_START_SCORING_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
			InverseInterpolator.forDouble(),
			InterpolationMap.interpolatorForRotation2d(),
			Map.of(1.5, Rotation2d.fromRotations(3), 6.0, Rotation2d.fromRotations(1))
		);

		FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_CONTINUE_SCORING_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
			InverseInterpolator.forDouble(),
			InterpolationMap.interpolatorForRotation2d(),
			Map.of(1.5, Rotation2d.fromRotations(3), 6.0, Rotation2d.fromRotations(1))
		);

		HOOD_POSITION_TOLERANCE_TO_START_SCORING_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
			InverseInterpolator.forDouble(),
			InterpolationMap.interpolatorForRotation2d(),
			Map.of(1.5, Rotation2d.fromDegrees(3), 6.0, Rotation2d.fromDegrees(1))
		);

		HOOD_POSITION_TOLERANCE_TO_CONTINUE_SCORING_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
			InverseInterpolator.forDouble(),
			InterpolationMap.interpolatorForRotation2d(),
			Map.of(1.5, Rotation2d.fromDegrees(3), 6.0, Rotation2d.fromDegrees(1))
		);
	}

}
