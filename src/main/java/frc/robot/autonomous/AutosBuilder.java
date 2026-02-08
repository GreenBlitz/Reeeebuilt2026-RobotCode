package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.statemachine.intakestatehandler.IntakeState;
import frc.robot.statemachine.shooterstatehandler.ShooterState;
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

	private static Supplier<PathPlannerAutoWrapper> getRightFirstQuarterAuto(Robot robot) {
		return () -> new PathPlannerAutoWrapper(
			new SequentialCommandGroup(
				firstQuarterAutoStart(robot, false),
				defaultDeadlineCommandWithPath(
					robot,
					PathHelper.PATH_PLANNER_PATHS.get("Right from middle to outpost"),
					() -> robot.getRobotCommander().scoreSequence()
				)
			),
			new Pose2d(),
			"Right path type 1"
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getLeftFirstQuarterAuto(Robot robot) {
		return () -> new PathPlannerAutoWrapper(
			new SequentialCommandGroup(
				firstQuarterAutoStart(robot, true),
				defaultDeadlineCommandWithPath(
					robot,
					PathHelper.PATH_PLANNER_PATHS.get("Left from middle to depot"),
					() -> robot.getRobotCommander().scoreSequence()
				)
			),
			new Pose2d(),
			"Left path type 1"
		);
	}

	public static List<Supplier<PathPlannerAutoWrapper>> getAutoList(Robot robot) {
		return List.of(getRightFirstQuarterAuto(robot), getLeftFirstQuarterAuto(robot));
	}

	private static Command resetSubsystemsAndIntakeAfter(Robot robot) {
		return new ParallelCommandGroup(
			robot.getRobotCommander().getShooterStateHandler().setState(ShooterState.RESET_SUBSYSTEMS),
			robot.getRobotCommander().getIntakeStateHandler().setState(IntakeState.RESET_FOUR_BAR)
		).andThen(robot.getRobotCommander().getIntakeStateHandler().setState(IntakeState.INTAKE));
	}

	private static Command firstQuarterAutoStart(Robot robot, boolean isPathLeft) {
		return defaultDeadlineCommandWithPath(
			robot,
			isPathLeft ? PathHelper.PATH_PLANNER_PATHS.get("Right intake on center line").mirrorPath() : PathHelper.PATH_PLANNER_PATHS.get("Right intake on center line"),
			() -> resetSubsystemsAndIntakeAfter(robot)
		);
	}

	private static Command defaultDeadlineCommandWithPath(Robot robot, PathPlannerPath deadline, Supplier<Command> command) {
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
			robot.getSwerve(),
			() -> robot.getPoseEstimator().getEstimatedPose(),
			deadline,
			AutonomousConstants.DEFAULT_PATH_CONSTRAINS,
			command,
			AutonomousConstants.DEFAULT_PATH_TOLERANCE
		);
	}

}
