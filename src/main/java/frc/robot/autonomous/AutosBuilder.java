package frc.robot.autonomous;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.utils.auto.PathHelper;
import frc.utils.auto.PathPlannerAutoWrapper;

import java.util.List;
import java.util.function.Supplier;

public class AutosBuilder {
	
	public static List<Supplier<PathPlannerAutoWrapper>> getAllTestAutos() {
		return List.of(
				() -> new PathPlannerAutoWrapper("Rotate"),
				() -> new PathPlannerAutoWrapper("Rotate 2m"),
				() -> new PathPlannerAutoWrapper("Straight 2m")
		);
	}
	
	public static List<Supplier<PathPlannerAutoWrapper>> getAutoList(
			Robot robot,
			Supplier<Command> intake,
			Supplier<Command> resetSubsystems,
			Supplier<Command> scoreSequence,
			PathConstraints pathfindingConstraints,
			Pose2d isNearEndOfPathTolerance
	) {
		return List.of(
				getLeftStartingToRightTroughNeutralZoneAuto(robot, intake, scoreSequence, resetSubsystems, pathfindingConstraints, isNearEndOfPathTolerance),
				getRightStartingToLeftTroughNeutralZoneAuto(robot, intake, scoreSequence, resetSubsystems, pathfindingConstraints, isNearEndOfPathTolerance)
		);
	}
	
	private static Supplier<PathPlannerAutoWrapper> getRightStartingToLeftTroughNeutralZoneAuto(
			Robot robot,
			Supplier<Command> intake,
			Supplier<Command> scoreSequence,
			Supplier<Command> resetSubsystems,
			PathConstraints pathfindingConstraints,
			Pose2d isNearEndOfPathTolerance
	) {
		return () -> new PathPlannerAutoWrapper(
				new SequentialCommandGroup(
						startingLineToNeutralZoneCenter(robot, intake, resetSubsystems, pathfindingConstraints, isNearEndOfPathTolerance, false),
						neutralZoneMiddleToStartingLine(robot, intake, scoreSequence, resetSubsystems, pathfindingConstraints, isNearEndOfPathTolerance, true)
				),
				new Pose2d(),
				"R starting - Neutral Zone - Left finish"
		);
	}
	
	private static Supplier<PathPlannerAutoWrapper> getLeftStartingToRightTroughNeutralZoneAuto(
			Robot robot,
			Supplier<Command> intake,
			Supplier<Command> scoreSequence,
			Supplier<Command> resetSubsystems,
			PathConstraints pathfindingConstraints,
			Pose2d isNearEndOfPathTolerance
	) {
		return () -> new PathPlannerAutoWrapper(
				new SequentialCommandGroup(
						startingLineToNeutralZoneCenter(robot, intake, resetSubsystems, pathfindingConstraints, isNearEndOfPathTolerance, true),
						neutralZoneMiddleToStartingLine(robot, intake, scoreSequence, resetSubsystems, pathfindingConstraints, isNearEndOfPathTolerance, false)
				),
				new Pose2d(),
				"L starting - Neutral Zone - right finish"
		);
	}
	
	private static Command startingLineToNeutralZoneCenter(
			Robot robot,
			Supplier<Command> intake,
			Supplier<Command> resetSubsystems,
			PathConstraints pathfindingConstraints,
			Pose2d isNearEndOfPathTolerance,
			boolean isLeft
	) {
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
				robot.getSwerve(),
				() -> robot.getPoseEstimator().getEstimatedPose(),
				isLeft
						? PathHelper.PATH_PLANNER_PATHS.get("R starting - Neutral zone center").mirrorPath()
						: PathHelper.PATH_PLANNER_PATHS.get("R starting - Neutral zone center"),
				pathfindingConstraints,
				() -> resetSubsystems.get().andThen(intake.get()),
				isNearEndOfPathTolerance
		);
	}
	
	private static Command neutralZoneMiddleToStartingLine(
			Robot robot,
			Supplier<Command> intake,
			Supplier<Command> resetSubsystems,
			Supplier<Command> scoreSequence,
			PathConstraints pathfindingConstraints,
			Pose2d isNearEndOfPathTolerance,
			boolean isLeft
	) {
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
				robot.getSwerve(),
				() -> robot.getPoseEstimator().getEstimatedPose(),
				isLeft
						? PathHelper.PATH_PLANNER_PATHS.get("Neutral zone center - Left starting line")
						: PathHelper.PATH_PLANNER_PATHS.get("Neutral zone center - Left starting line").mirrorPath(),
				pathfindingConstraints,
				() -> resetSubsystems.get().andThen(new ParallelCommandGroup(intake.get(), scoreSequence.get())),
				isNearEndOfPathTolerance
		);
	}
	
}