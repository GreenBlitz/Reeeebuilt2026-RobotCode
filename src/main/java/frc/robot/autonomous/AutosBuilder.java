package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.statemachine.intakestatehandler.IntakeState;
import frc.robot.statemachine.shooterstatehandler.ShooterState;
import frc.robot.subsystems.swerve.Swerve;
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

	public static List<Supplier<PathPlannerAutoWrapper>> getAutoList(Robot robot,Supplier<Command> intake,Supplier<Command> scoreSequence) {
		return List.of(getRightFirstQuarterAuto(robot,scoreSequence,intake), getLeftFirstQuarterAuto(robot,scoreSequence,intake));
	}

	private static Supplier<PathPlannerAutoWrapper> getRightFirstQuarterAuto(Robot robot,Supplier<Command> scoreSequence, Supplier<Command> intake) {
		return () -> new PathPlannerAutoWrapper(
			new SequentialCommandGroup(
				rightStartingLineToRightMiddleWith(robot, intake,false),
				rightMiddleToOutpostWithScoring(robot,scoreSequence)
			),
			new Pose2d(),
			"R starting - R mid - Outpost"
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getLeftFirstQuarterAuto(Robot robot,Supplier<Command> scoreSequence,Supplier<Command> intake) {
		return () -> new PathPlannerAutoWrapper(
			new SequentialCommandGroup(
				rightStartingLineToRightMiddleWith(robot, intake,true),
				leftMiddleToDepotWithScoring(robot,scoreSequence)
			),
			new Pose2d(),
			"L starting - L mid - Depot"
		);
	}

	private static Command rightStartingLineToRightMiddleWith(Robot robot, Supplier<Command> intake, boolean isMirrored) {
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
				robot.getSwerve(),
				() -> robot.getPoseEstimator().getEstimatedPose(),
		isMirrored
				? PathHelper.PATH_PLANNER_PATHS.get("R starting - R mid").mirrorPath()
				: PathHelper.PATH_PLANNER_PATHS.get("R starting - R mid"),
				AutonomousConstants.DEFAULT_PATH_CONSTRAINS,
				intake,
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
