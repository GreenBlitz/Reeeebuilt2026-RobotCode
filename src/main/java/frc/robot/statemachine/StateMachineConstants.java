package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.utils.math.FieldMath;

public class StateMachineConstants {

	public static final Rotation2d TURRET_TOLERANCE_TO_START_SCORING = Rotation2d.fromDegrees(3);
	public static final Rotation2d TURRET_TOLERANCE_TO_CONTINUE_SCORING = Rotation2d.fromDegrees(3);
	public final static Rotation2d FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_START_SCORING = Rotation2d.fromRotations(1);
	public final static Rotation2d FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_CONTINUE_SCORING = Rotation2d.fromRotations(1);
	public static final Rotation2d HOOD_POSITION_TOLERANCE_TO_START_SCORING = Rotation2d.fromDegrees(5);
	public static final Rotation2d HOOD_POSITION_TOLERANCE_TO_CONTINUE_SCORING = Rotation2d.fromDegrees(5);
	public static final Rotation2d TURRET_TOLERANCE_TO_START_PASSING = Rotation2d.fromDegrees(3);
	public static final Rotation2d TURRET_TOLERANCE_TO_CONTINUE_PASSING = Rotation2d.fromDegrees(3);
	public final static Rotation2d FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_START_PASSING = Rotation2d.fromRotations(1);
	public final static Rotation2d FLYWHEEL_VELOCITY_TOLERANCE_RPS_TO_CONTINUE_PASSING = Rotation2d.fromRotations(1);
	public static final Rotation2d HOOD_POSITION_TOLERANCE_TO_START_PASSING = Rotation2d.fromDegrees(5);
	public static final Rotation2d HOOD_POSITION_TOLERANCE_TO_CONTINUE_PASSING = Rotation2d.fromDegrees(5);
	public static final Rotation2d MAX_ANGLE_FROM_GOAL_CENTER = Rotation2d.fromDegrees(70);
	public static final double MAX_DISTANCE_TO_SCORE_METERS = 6;
	public static final double MAX_DISTANCE_TO_PASS_METERS = 6;
	public static final double TIME_FOR_TRAIN_TO_ACCELERATE_SECONDS = 0.05;
	public static final int DEGREES_OF_OVERSHOOT_FOR_AIM_AT_HUB_ASSIST = 5;
	private static final double MIN_X_VALUE_FOR_BEHIND_HUB_PASSING = 6;
	private static final double TARGET_X_VALUE_FOR_PASSING = Field.getAllianceRelative(Field.getHubMiddle()).getX() - 1;

	private static final Translation2d OUTPOST_PRESET_PASSING_TARGET = new Translation2d(1, 1);
	private static final Translation2d DEPOT_PRESET_PASSING_TARGET = FieldMath.mirror(OUTPOST_PRESET_PASSING_TARGET, false, true);

	public static double getTargetXValueForPassing() {
		if (!Field.isFieldConventionAlliance()) {
			return FieldMath.mirrorX(TARGET_X_VALUE_FOR_PASSING);
		}
		return TARGET_X_VALUE_FOR_PASSING;
	}

	public static Translation2d getOutpostPresetPassingTarget() {
		return Field.getAllianceRelative(OUTPOST_PRESET_PASSING_TARGET);
	}

	public static Translation2d getDepotPresetPassingTarget() {
		return Field.getAllianceRelative(DEPOT_PRESET_PASSING_TARGET);
	}

	public static double getMinXValueForBehindHubPassing() {
		if (!Field.isFieldConventionAlliance()) {
			return FieldMath.mirrorX(MIN_X_VALUE_FOR_BEHIND_HUB_PASSING);
		}
		return MIN_X_VALUE_FOR_BEHIND_HUB_PASSING;
	}

}
