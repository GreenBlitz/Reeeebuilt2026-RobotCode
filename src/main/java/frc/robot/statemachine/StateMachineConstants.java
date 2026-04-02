package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.utils.math.FieldMath;

public class StateMachineConstants {

	public static final Rotation2d TURRET_TOLERANCE_TO_START_SCORING = Rotation2d.fromDegrees(6);
	public static final Rotation2d TURRET_TOLERANCE_TO_CONTINUE_SCORING = Rotation2d.fromDegrees(6);
	public final static Rotation2d FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_START_SCORING = Rotation2d.fromRotations(3);
	public final static Rotation2d FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_CONTINUE_SCORING = Rotation2d.fromRotations(12);
	public static final Rotation2d HOOD_POSITION_TOLERANCE_TO_START_SCORING = Rotation2d.fromDegrees(3.5);
	public static final Rotation2d HOOD_POSITION_TOLERANCE_TO_CONTINUE_SCORING = Rotation2d.fromDegrees(3.5);

	public static final Rotation2d TURRET_TOLERANCE_TO_START_PASSING = Rotation2d.fromDegrees(6);
	public static final Rotation2d TURRET_TOLERANCE_TO_CONTINUE_PASSING = Rotation2d.fromDegrees(6);
	public final static Rotation2d FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_START_PASSING = Rotation2d.fromRotations(12);
	public final static Rotation2d FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_CONTINUE_PASSING = Rotation2d.fromRotations(12);
	public static final Rotation2d HOOD_POSITION_TOLERANCE_TO_START_PASSING = Rotation2d.fromDegrees(5);
	public static final Rotation2d HOOD_POSITION_TOLERANCE_TO_CONTINUE_PASSING = Rotation2d.fromDegrees(5);

	public static final double MAX_DISTANCE_TO_SCORE_METERS = 8.5;
	public static final double MAX_DISTANCE_TO_PASS_METERS = 15;

	public static final double TIME_FOR_MAGAZINE_TO_ACCELERATE_SECONDS = 0.05;

	public static final int DEGREES_OF_OVERSHOOT_FOR_AIM_AT_HUB_ASSIST = 5;

	public static final double PASSING_NEAR_HUB_SAFETY_BUFFER_METERS = 0.25;
	public static final Translation2d OUTPOST_PRESET_PASSING_TARGET = new Translation2d(1.5, 1.25);
	public static final Translation2d DEPOT_PRESET_PASSING_TARGET = FieldMath.mirror(OUTPOST_PRESET_PASSING_TARGET, false, true);

	public static final double MAX_TIMES_TO_CALCULATE_PREDICTED_TURRET_POSE_BY_FLIGHT_TIME = 20;
	public static final double MIN_DIFFERENCE_BETWEEN_FLIGHT_TIMES_TO_STOP_CALCULATIONS_SECONDS = 0.03;

	public static Translation2d getOutpostPresetPassingTarget() {
		return Field.getAllianceRelative(OUTPOST_PRESET_PASSING_TARGET);
	}

	public static Translation2d getDepotPresetPassingTarget() {
		return Field.getAllianceRelative(DEPOT_PRESET_PASSING_TARGET);
	}

}
