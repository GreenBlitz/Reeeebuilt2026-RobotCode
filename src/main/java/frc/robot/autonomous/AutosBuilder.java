package frc.robot.autonomous;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.statemachine.intakestatehandler.IntakeState;
import frc.robot.statemachine.shooterstatehandler.ShooterState;
import frc.utils.auto.PathHelper;
import frc.utils.auto.PathPlannerAutoWrapper;

import java.util.List;
import java.util.function.Supplier;

public class AutosBuilder {

	private static final PathConstraints DEFAULT_PATH_CONSTRAINS = new PathConstraints(
		4.200,
		2.000,
		Rotation2d.fromDegrees(540).getRadians(),
		Rotation2d.fromDegrees(720).getRadians()
	);

	private static final Pose2d DEFAULT_PATH_TOLERANCE = new Pose2d(0.3, 0.3, Rotation2d.fromDegrees(1));

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
				PathFollowingCommandsBuilder.deadlineCommandWithPath(
					robot.getSwerve(),
					() -> robot.getPoseEstimator().getEstimatedPose(),
					PathHelper.PATH_PLANNER_PATHS.get("Right start 1-1"),
					DEFAULT_PATH_CONSTRAINS,
					() -> new ParallelCommandGroup(
						robot.getRobotCommander().getShooterStateHandler().setState(ShooterState.RESET_SUBSYSTEMS),
						robot.getRobotCommander().getIntakeStateHandler().setState(IntakeState.RESET_FOUR_BAR)
					).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
						.andThen(robot.getRobotCommander().getIntakeStateHandler().setState(IntakeState.INTAKE)),
					DEFAULT_PATH_TOLERANCE
				),
				PathFollowingCommandsBuilder.deadlineCommandWithPath(
					robot.getSwerve(),
					() -> robot.getPoseEstimator().getEstimatedPose(),
					PathHelper.PATH_PLANNER_PATHS.get("Right 1-2"),
					DEFAULT_PATH_CONSTRAINS,
					() -> robot.getRobotCommander().scoreSequence(),
					DEFAULT_PATH_TOLERANCE
				)
			),
			new Pose2d(),
			"Right path type 1"
		);
	}


	public static List<Supplier<PathPlannerAutoWrapper>> getAutoList(Robot robot) {
		return List.of(getRightFirstQuarterAuto(robot));
	}


}
