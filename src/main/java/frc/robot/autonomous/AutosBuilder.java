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
			getRightStartingToRightMiddleToOutpostAuto(
				robot,
				scoreSequence,
				intake,
				resetSubsystems,
				pathfindingConstraints,
				isNearEndOfPathTolerance
			),
			getLeftStartingToLeftMiddleToDepotAuto(
				robot,
				scoreSequence,
				intake,
				resetSubsystems,
				pathfindingConstraints,
				isNearEndOfPathTolerance
			),
			getLeftStartingToRightTroughNeutralZoneAuto(robot,pathfindingConstraints,isNearEndOfPathTolerance),
			getRightStartingToRightTroughNeutralZoneAuto(robot,pathfindingConstraints,isNearEndOfPathTolerance)
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getRightStartingToRightMiddleToOutpostAuto(
		Robot robot,
		Supplier<Command> scoreSequence,
		Supplier<Command> intake,
		Supplier<Command> resetSubsystems,
		PathConstraints pathfindingConstraints,
		Pose2d isNearEndOfPathTolerance
	) {
		return () -> new PathPlannerAutoWrapper(
			new SequentialCommandGroup(
				startingLineToMiddle(robot, intake, resetSubsystems, pathfindingConstraints, isNearEndOfPathTolerance, false),
				rightMiddleToOutpostWithScoring(robot, scoreSequence, pathfindingConstraints, isNearEndOfPathTolerance)
			),
			new Pose2d(),
			"R starting - R mid - Outpost"
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getLeftStartingToLeftMiddleToDepotAuto(
		Robot robot,
		Supplier<Command> scoreSequence,
		Supplier<Command> intake,
		Supplier<Command> resetSubsystems,
		PathConstraints pathfindingConstraints,
		Pose2d isNearEndOfPathTolerance
	) {
		return () -> new PathPlannerAutoWrapper(
			new SequentialCommandGroup(
				startingLineToMiddle(robot, intake, resetSubsystems, pathfindingConstraints, isNearEndOfPathTolerance, true),
				leftMiddleToDepotWithScoring(robot, scoreSequence, pathfindingConstraints, isNearEndOfPathTolerance)
			),
			new Pose2d(),
			"L starting - L mid - Depot"
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getRightStartingToRightTroughNeutralZoneAuto(
		Robot robot,
		PathConstraints pathfindingConstraints,
		Pose2d isNearEndOfPathTolerance
	) {
		return () -> new PathPlannerAutoWrapper(
			moveSidesThroughNeutralZoneWithIntakeAndScoring(robot,pathfindingConstraints,isNearEndOfPathTolerance,false),
			new Pose2d(),
			"R starting - Neutral Zone - Left finish"
		);
	}
	private static Supplier<PathPlannerAutoWrapper> getLeftStartingToRightTroughNeutralZoneAuto(
		Robot robot,
		PathConstraints pathfindingConstraints,
		Pose2d isNearEndOfPathTolerance
	) {
		return () -> new PathPlannerAutoWrapper(
			moveSidesThroughNeutralZoneWithIntakeAndScoring(robot,pathfindingConstraints,isNearEndOfPathTolerance,true),
			new Pose2d(),
			"L starting - Neutral Zone - Right finish"
		);
	}

	private static Command startingLineToMiddle(
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
				? PathHelper.PATH_PLANNER_PATHS.get("R starting - R mid").mirrorPath()
				: PathHelper.PATH_PLANNER_PATHS.get("R starting - R mid"),
			pathfindingConstraints,
			() -> resetSubsystems.get().andThen(intake.get()),
			isNearEndOfPathTolerance
		);
	}
	private static Command moveSidesThroughNeutralZoneWithIntakeAndScoring(
		Robot robot,
		PathConstraints pathfindingConstraints,
		Pose2d isNearEndOfPathTolerance,
		boolean isLeft
	) {
		return PathFollowingCommandsBuilder.followAdjustedPathThenStop(
			robot.getSwerve(),
			() -> robot.getPoseEstimator().getEstimatedPose(),
			isLeft
				? PathHelper.PATH_PLANNER_PATHS.get("R starting - Neutral zone intake - Left shoot").mirrorPath()
				: PathHelper.PATH_PLANNER_PATHS.get("R starting - Neutral zone intake - Left shoot"),
				pathfindingConstraints,
				isNearEndOfPathTolerance
		);
	}

	private static Command rightMiddleToOutpostWithScoring(
		Robot robot,
		Supplier<Command> scoreSequence,
		PathConstraints pathfindingConstraints,
		Pose2d isNearEndOfPathTolerance
	) {
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
			robot.getSwerve(),
			() -> robot.getPoseEstimator().getEstimatedPose(),
			PathHelper.PATH_PLANNER_PATHS.get("R mid - Outpost"),
			pathfindingConstraints,
			scoreSequence,
			isNearEndOfPathTolerance
		);
	}

	private static Command leftMiddleToDepotWithScoring(
		Robot robot,
		Supplier<Command> scoreSequence,
		PathConstraints pathfindingConstraints,
		Pose2d isNearEndOfPathTolerance
	) {
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
			robot.getSwerve(),
			() -> robot.getPoseEstimator().getEstimatedPose(),
			PathHelper.PATH_PLANNER_PATHS.get("L mid - Depot"),
			pathfindingConstraints,
			scoreSequence,
			isNearEndOfPathTolerance
		);
	}

}
