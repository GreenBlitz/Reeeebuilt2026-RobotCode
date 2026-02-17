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
		Supplier<Command> intake,
		Supplier<Command> resetSubsystems,
		Supplier<Command> scoreSequence,
		PathConstraints pathfindingConstraints,
		Pose2d isNearEndOfPathTolerance
	) {
		return List.of(
			getStartingSideToMiddleSideToAllianceSideAuto(
				robot,
				scoreSequence,
				intake,
				resetSubsystems,
				pathfindingConstraints,
				isNearEndOfPathTolerance,
				AllianceSide.OUTPOST
			),
			getStartingSideToMiddleSideToAllianceSideAuto(
				robot,
				scoreSequence,
				intake,
				resetSubsystems,
				pathfindingConstraints,
				isNearEndOfPathTolerance,
				AllianceSide.DEPOT
			)
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getStartingSideToMiddleSideToAllianceSideAuto(
		Robot robot,
		Supplier<Command> scoreSequence,
		Supplier<Command> intake,
		Supplier<Command> resetSubsystems,
		PathConstraints pathfindingConstraints,
		Pose2d isNearEndOfPathTolerance,
		AllianceSide side
	) {
		return () -> new PathPlannerAutoWrapper(
			new SequentialCommandGroup(
				startingLineToMiddle(robot, intake, resetSubsystems, pathfindingConstraints, isNearEndOfPathTolerance, side),
				sideMiddleToSideWithScoring(robot, scoreSequence, pathfindingConstraints, isNearEndOfPathTolerance, side)
			),
			new Pose2d(),
			side == AllianceSide.OUTPOST ? "R starting - R mid - Outpost" : "L starting - L mid - Depot"
		);
	}

	private static Command startingLineToMiddle(
		Robot robot,
		Supplier<Command> intake,
		Supplier<Command> resetSubsystems,
		PathConstraints pathfindingConstraints,
		Pose2d isNearEndOfPathTolerance,
		AllianceSide side
	) {
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
			robot.getSwerve(),
			() -> robot.getPoseEstimator().getEstimatedPose(),
			side == AllianceSide.DEPOT
				? PathHelper.PATH_PLANNER_PATHS.get("R starting - R mid").mirrorPath()
				: PathHelper.PATH_PLANNER_PATHS.get("R starting - R mid"),
			pathfindingConstraints,
			() -> resetSubsystems.get().andThen(intake.get()),
			isNearEndOfPathTolerance
		);
	}

	private static Command sideMiddleToSideWithScoring(
		Robot robot,
		Supplier<Command> scoreSequence,
		PathConstraints pathfindingConstraints,
		Pose2d isNearEndOfPathTolerance,
		AllianceSide side
	) {
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
			robot.getSwerve(),
			() -> robot.getPoseEstimator().getEstimatedPose(),
			side == AllianceSide.OUTPOST
				? PathHelper.PATH_PLANNER_PATHS.get("R mid - Outpost")
				: PathHelper.PATH_PLANNER_PATHS.get("L mid - Depot"),
			pathfindingConstraints,
			scoreSequence,
			isNearEndOfPathTolerance
		);
	}

}
