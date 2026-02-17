package frc.robot.autonomous;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.constants.field.AllianceSide;
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
		Supplier<Command> resetSubsystems,
		Supplier<Command> intake,
		Supplier<Command> scoreSequence,
		PathConstraints pathfindingConstraints,
		Pose2d isNearEndOfPathTolerance
	) {
		return List.of(
			getStartingLineToMiddleToAllianceSideAuto(
				robot,
				resetSubsystems,
				intake,
				scoreSequence,
				pathfindingConstraints,
				isNearEndOfPathTolerance,
				AllianceSide.OUTPOST
			),
			getStartingLineToMiddleToAllianceSideAuto(
				robot,
				resetSubsystems,
				intake,
				scoreSequence,
				pathfindingConstraints,
				isNearEndOfPathTolerance,
				AllianceSide.DEPOT
			)
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getStartingLineToMiddleToAllianceSideAuto(
		Robot robot,
		Supplier<Command> resetSubsystems,
		Supplier<Command> intake,
		Supplier<Command> scoreSequence,
		PathConstraints pathfindingConstraints,
		Pose2d isNearEndOfPathTolerance,
		AllianceSide startingSide
	) {
		return () -> new PathPlannerAutoWrapper(
			new SequentialCommandGroup(
				startingLineToMiddleCommand(robot, resetSubsystems, intake, pathfindingConstraints, isNearEndOfPathTolerance, startingSide),
				middleToAllienceSideWithScoringCommand(robot, scoreSequence, pathfindingConstraints, isNearEndOfPathTolerance, startingSide)
			),
			new Pose2d(),
			startingSide == AllianceSide.OUTPOST ? "R starting - R mid - Outpost" : "L starting - L mid - Depot"
		);
	}

	private static Command startingLineToMiddleCommand(
		Robot robot,
		Supplier<Command> resetSubsystems,
		Supplier<Command> intake,
		PathConstraints pathfindingConstraints,
		Pose2d isNearEndOfPathTolerance,
		AllianceSide startingSide
	) {
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
			robot.getSwerve(),
			() -> robot.getPoseEstimator().getEstimatedPose(),
			startingSide == AllianceSide.DEPOT
				? PathHelper.PATH_PLANNER_PATHS.get("R starting - R mid").mirrorPath()
				: PathHelper.PATH_PLANNER_PATHS.get("R starting - R mid"),
			pathfindingConstraints,
			() -> resetSubsystems.get().andThen(intake.get()),
			isNearEndOfPathTolerance
		);
	}

	private static Command middleToAllienceSideWithScoringCommand(
		Robot robot,
		Supplier<Command> scoreSequence,
		PathConstraints pathfindingConstraints,
		Pose2d isNearEndOfPathTolerance,
		AllianceSide startingSide
	) {
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
			robot.getSwerve(),
			() -> robot.getPoseEstimator().getEstimatedPose(),
			startingSide == AllianceSide.OUTPOST
				? PathHelper.PATH_PLANNER_PATHS.get("R mid - Outpost")
				: PathHelper.PATH_PLANNER_PATHS.get("L mid - Depot"),
			pathfindingConstraints,
			scoreSequence,
			isNearEndOfPathTolerance
		);
	}

}
