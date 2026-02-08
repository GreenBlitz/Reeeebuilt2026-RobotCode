package frc.robot.autonomous;

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
		Supplier<Command> scoreSequence
	) {
		return List.of(
			getRightStartingToRightMiddleToOutpostAuto(robot, scoreSequence, intake, resetSubsystems),
			getLeftStartingToLeftMiddleToDepotAuto(robot, scoreSequence, intake, resetSubsystems)
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getRightStartingToRightMiddleToOutpostAuto(
		Robot robot,
		Supplier<Command> scoreSequence,
		Supplier<Command> intake,
		Supplier<Command> resetSubsystems
	) {
		return () -> new PathPlannerAutoWrapper(
			new SequentialCommandGroup(
				rightStartingLineToRightMiddle(robot, intake, resetSubsystems, false),
				rightMiddleToOutpostWithScoring(robot, scoreSequence)
			),
			new Pose2d(),
			"R starting - R mid - Outpost"
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getLeftStartingToLeftMiddleToDepotAuto(
		Robot robot,
		Supplier<Command> scoreSequence,
		Supplier<Command> intake,
		Supplier<Command> resetSubsystems
	) {
		return () -> new PathPlannerAutoWrapper(
			new SequentialCommandGroup(
				rightStartingLineToRightMiddle(robot, intake, resetSubsystems, true),
				leftMiddleToDepotWithScoring(robot, scoreSequence)
			),
			new Pose2d(),
			"L starting - L mid - Depot"
		);
	}

	private static Command rightStartingLineToRightMiddle(
		Robot robot,
		Supplier<Command> intake,
		Supplier<Command> resetSubsystems,
		boolean isMirrored
	) {
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
			robot.getSwerve(),
			() -> robot.getPoseEstimator().getEstimatedPose(),
			isMirrored
				? PathHelper.PATH_PLANNER_PATHS.get("R starting - R mid").mirrorPath()
				: PathHelper.PATH_PLANNER_PATHS.get("R starting - R mid"),
			AutonomousConstants.DEFAULT_PATH_CONSTRAINS,
			() -> resetSubsystems.get().andThen(intake.get()),
			AutonomousConstants.DEFAULT_PATH_TOLERANCE
		);
	}

	private static Command rightMiddleToOutpostWithScoring(Robot robot, Supplier<Command> scoreSequence) {
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
			robot.getSwerve(),
			() -> robot.getPoseEstimator().getEstimatedPose(),
			PathHelper.PATH_PLANNER_PATHS.get("R mid - Outpost"),
			AutonomousConstants.DEFAULT_PATH_CONSTRAINS,
			scoreSequence,
			AutonomousConstants.DEFAULT_PATH_TOLERANCE
		);
	}

	private static Command leftMiddleToDepotWithScoring(Robot robot, Supplier<Command> scoreSequence) {
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
			robot.getSwerve(),
			() -> robot.getPoseEstimator().getEstimatedPose(),
			PathHelper.PATH_PLANNER_PATHS.get("L mid - Depot"),
			AutonomousConstants.DEFAULT_PATH_CONSTRAINS,
			scoreSequence,
			AutonomousConstants.DEFAULT_PATH_TOLERANCE
		);
	}

}
