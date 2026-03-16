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

	private static boolean hasPathEnded = false;

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
		Supplier<Command> openIntake,
		Supplier<Command> closeIntake,
		Supplier<Command> scoreSequence,
		PathConstraints pathfindingConstraints,
		Pose2d regularIsNearEndOfPathTolerance,
		Pose2d stuckIsNearEndOfPathTolerance,
		double stuckDebounceSeconds
	) {
		return List.of(
			getQuarterAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.OUTPOST
			),
			getQuarterAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.DEPOT
			)
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getQuarterAuto(
		Robot robot,
		Supplier<Command> resetSubsystems,
		Supplier<Command> openIntake,
		Supplier<Command> closeIntake,
		Supplier<Command> scoreSequence,
		PathConstraints pathfindingConstraints,
		Pose2d regularIsNearEndOfPathTolerance,
		Pose2d stuckIsNearEndOfPathTolerance,
		double stuckDebounceSeconds,
		AllianceSide startingSide
	) {
		hasPathEnded = false;
		return () -> new PathPlannerAutoWrapper(
			new SequentialCommandGroup(
				new ParallelDeadlineGroup(
					new SequentialCommandGroup(
						new WaitUntilCommand(() -> hasPathEnded),
						new WaitCommand(AutonomousConstants.TIME_TO_EMPTY_TANK_AFTER_END_OF_PATH)
					),
					new SequentialCommandGroup(
						PathFollowingCommandsBuilder
							.followAdjustedPathThenStop(
								robot.getSwerve(),
								() -> robot.getPoseEstimator().getEstimatedPose(),
								startingSide == AllianceSide.DEPOT
									? PathHelper.PATH_PLANNER_PATHS.get("L quarter")
									: PathHelper.PATH_PLANNER_PATHS.get("R quarter"),
								pathfindingConstraints,
								regularIsNearEndOfPathTolerance,
								stuckIsNearEndOfPathTolerance,
								stuckDebounceSeconds,
								robot.getSwerve().getLogPath()
							)
							.asProxy()
							.alongWith(new InstantCommand(() -> hasPathEnded = false))
							.andThen(new InstantCommand(() -> hasPathEnded = true)),
						new SequentialCommandGroup(
							resetSubsystems.get(),
							new ParallelDeadlineGroup(
								new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_START_SHOOTING_AFTER_AUTO_START)
									.andThen(scoreSequence.get()),
								openIntake.get()
									.until(() -> hasPathEnded)
									.andThen(
										new ParallelCommandGroup(
											new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_START_WIGGLE_AFTER_PATH_END).andThen(
												robot.getSwerve()
													.getCommandsBuilder()
													.wiggle(AutonomousConstants.WIGGLE_RANGE, AutonomousConstants.TIME_BETWEEN_WIGGLES_SECONDS)
											).asProxy(),
											new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_CLOSE_INTAKE_AFTER_PATH_END_SECONDS)
												.andThen(closeIntake.get())
										)
									)
							)
						)
					)
				),
				doubleSwipeCommand(
					robot,
					resetSubsystems,
					openIntake,
					closeIntake,
					scoreSequence,
					pathfindingConstraints,
					regularIsNearEndOfPathTolerance,
					stuckIsNearEndOfPathTolerance,
					stuckDebounceSeconds,
					startingSide
				)
			),
			new Pose2d(),
			startingSide == AllianceSide.OUTPOST ? "R quarter" : "L quarter"
		);
	}

	private static Command doubleSwipeCommand(
		Robot robot,
		Supplier<Command> resetSubsystems,
		Supplier<Command> openIntake,
		Supplier<Command> closeIntake,
		Supplier<Command> scoreSequence,
		PathConstraints pathfindingConstraints,
		Pose2d regularIsNearEndOfPathTolerance,
		Pose2d stuckIsNearEndOfPathTolerance,
		double stuckDebounceSeconds,
		AllianceSide allianceSide
	) {
		return new ParallelCommandGroup(
			PathFollowingCommandsBuilder
				.followAdjustedPathThenStop(
					robot.getSwerve(),
					() -> robot.getPoseEstimator().getEstimatedPose(),
					allianceSide == AllianceSide.DEPOT
						? PathHelper.PATH_PLANNER_PATHS.get("L double swipe")
						: PathHelper.PATH_PLANNER_PATHS.get("R double swipe"),
					pathfindingConstraints,
					regularIsNearEndOfPathTolerance,
					stuckIsNearEndOfPathTolerance,
					stuckDebounceSeconds,
					robot.getSwerve().getLogPath()

				)
				.asProxy()
				.alongWith(new InstantCommand(() -> hasPathEnded = false))
				.andThen(new InstantCommand(() -> hasPathEnded = true)),
			new SequentialCommandGroup(
				resetSubsystems.get(),
				new ParallelCommandGroup(
					new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_START_SHOOTING_AFTER_AUTO_START).andThen(scoreSequence.get()),
					openIntake.get()
						.until(() -> hasPathEnded)
						.andThen(
							new ParallelCommandGroup(
								new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_START_WIGGLE_AFTER_PATH_END)
									.andThen(
										robot.getSwerve()
											.getCommandsBuilder()
											.wiggle(AutonomousConstants.WIGGLE_RANGE, AutonomousConstants.TIME_BETWEEN_WIGGLES_SECONDS)
									)
									.asProxy(),
								new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_CLOSE_INTAKE_AFTER_PATH_END_SECONDS)
									.andThen(closeIntake.get())
							)
						)
				)
			)
		);
	}

}
