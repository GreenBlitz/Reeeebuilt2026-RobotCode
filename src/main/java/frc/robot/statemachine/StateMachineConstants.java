package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Rotation2d;

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
	public static final double SECONDS_TO_WAIT_AFTER_SHOOT = 0.2;
	public static final double TIME_FOR_TRAIN_TO_ACCELERATE_SECONDS = 0.05;
	public static final int DEGREES_OF_OVERSHOOT_FOR_AIM_AT_HUB_ASSIST = 5;

}
