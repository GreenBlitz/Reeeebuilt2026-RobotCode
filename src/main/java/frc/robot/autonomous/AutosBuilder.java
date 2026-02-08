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
		PathConstraints constraints,
		Pose2d tolerance
	) {
		return List.of(
			getRightStartingToRightMiddleToOutpostAuto(robot, scoreSequence, intake, resetSubsystems,constraints,tolerance),
			getLeftStartingToLeftMiddleToDepotAuto(robot, scoreSequence, intake, resetSubsystems,constraints,tolerance)
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getRightStartingToRightMiddleToOutpostAuto(
		Robot robot,
		Supplier<Command> scoreSequence,
		Supplier<Command> intake,
		Supplier<Command> resetSubsystems,
		PathConstraints constraints,
		Pose2d tolerance
	) {
		return () -> new PathPlannerAutoWrapper(
			new SequentialCommandGroup(
				rightStartingLineToRightMiddle(robot, intake, resetSubsystems, constraints,tolerance,false),
				rightMiddleToOutpostWithScoring(robot, scoreSequence,constraints,tolerance)
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
		PathConstraints constraints,
		Pose2d tolerance
	) {
		return () -> new PathPlannerAutoWrapper(
			new SequentialCommandGroup(
				rightStartingLineToRightMiddle(robot, intake, resetSubsystems,constraints,tolerance, true),
				leftMiddleToDepotWithScoring(robot, scoreSequence,constraints,tolerance)
			),
			new Pose2d(),
			"L starting - L mid - Depot"
		);
	}

	private static Command rightStartingLineToRightMiddle(
		Robot robot,
		Supplier<Command> intake,
		Supplier<Command> resetSubsystems,
		PathConstraints constraints,
		Pose2d tolerance,
		boolean isMirrored
	) {
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
			robot.getSwerve(),
			() -> robot.getPoseEstimator().getEstimatedPose(),
			isMirrored
				? PathHelper.PATH_PLANNER_PATHS.get("R starting - R mid").mirrorPath()
				: PathHelper.PATH_PLANNER_PATHS.get("R starting - R mid"),
			constraints,
			() -> resetSubsystems.get().andThen(intake.get()),
			tolerance
		);
	}

	private static Command rightMiddleToOutpostWithScoring(Robot robot, Supplier<Command> scoreSequence, PathConstraints constraints, Pose2d tolerance) {
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
			robot.getSwerve(),
			() -> robot.getPoseEstimator().getEstimatedPose(),
			PathHelper.PATH_PLANNER_PATHS.get("R mid - Outpost"),
			constraints,
			scoreSequence,
			tolerance
		);
	}

	private static Command leftMiddleToDepotWithScoring(Robot robot, Supplier<Command> scoreSequence, PathConstraints constraints, Pose2d tolerance) {
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
			robot.getSwerve(),
			() -> robot.getPoseEstimator().getEstimatedPose(),
			PathHelper.PATH_PLANNER_PATHS.get("L mid - Depot"),
			constraints,
			scoreSequence,
			tolerance
		);
	}

}
