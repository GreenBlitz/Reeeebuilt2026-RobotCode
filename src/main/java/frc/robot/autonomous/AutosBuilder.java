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

	public static boolean hasPathEnded = false;

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
		Supplier<Command> passSequence,
		Supplier<Command> outtakeSequence,
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
				AllianceSide.DEPOT
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
				AllianceSide.OUTPOST
			),
			getLightQuarterAuto(
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
			),
			getLightQuarterAuto(
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
			getExtendedLQuarterAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds
			),
			getHorseshoeAuto(
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
			),
			getHorseshoeAuto(
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
			getPushAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				passSequence,
				outtakeSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds
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
		return () -> new PathPlannerAutoWrapper(
			new ParallelCommandGroup(
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
												.withDeadline(new WaitCommand(AutonomousConstants.TIME_TO_WAIT_AT_DEPOT))
										)
										.andThen(
											getAllianceSideToStartingLineAuto(
												robot,
												AllianceSide.DEPOT,
												pathfindingConstraints,
												regularIsNearEndOfPathTolerance,
												stuckIsNearEndOfPathTolerance,
												stuckDebounceSeconds
											)
										)
										.asProxy(),
									new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_CLOSE_INTAKE_AFTER_PATH_END_SECONDS)
										.andThen(closeIntake.get())
								)
							)
					)
				)
			),
			new Pose2d(),
			startingSide == AllianceSide.DEPOT ? "L quarter" : "R quarter"
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getLightQuarterAuto(
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
		return () -> new PathPlannerAutoWrapper(
			new ParallelCommandGroup(
				PathFollowingCommandsBuilder
					.followAdjustedPathThenStop(
						robot.getSwerve(),
						() -> robot.getPoseEstimator().getEstimatedPose(),
						startingSide == AllianceSide.DEPOT
							? PathHelper.PATH_PLANNER_PATHS.get("L quarter light")
							: PathHelper.PATH_PLANNER_PATHS.get("R quarter light"),
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
												.withDeadline(new WaitCommand(AutonomousConstants.TIME_TO_WAIT_AT_DEPOT))
										)
										.andThen(
											getAllianceSideToStartingLineAuto(
												robot,
												startingSide,
												pathfindingConstraints,
												regularIsNearEndOfPathTolerance,
												stuckIsNearEndOfPathTolerance,
												stuckDebounceSeconds
											)
										)
										.asProxy(),
									new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_CLOSE_INTAKE_AFTER_PATH_END_SECONDS)
										.andThen(closeIntake.get())
								)
							)
					)
				)
			),
			new Pose2d(),
			startingSide == AllianceSide.DEPOT ? "L quarter light" : "R quarter light"
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getExtendedLQuarterAuto(
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
		return () -> new PathPlannerAutoWrapper(
			new ParallelCommandGroup(
				PathFollowingCommandsBuilder
					.followAdjustedPathThenStop(
						robot.getSwerve(),
						() -> robot.getPoseEstimator().getEstimatedPose(),
						PathHelper.PATH_PLANNER_PATHS.get("L quarter to outpost"),
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
												.withDeadline(new WaitCommand(AutonomousConstants.TIME_TO_WAIT_AT_DEPOT))
										)
										.andThen(
											getAllianceSideToStartingLineAuto(
												robot,
												AllianceSide.OUTPOST,
												pathfindingConstraints,
												regularIsNearEndOfPathTolerance,
												stuckIsNearEndOfPathTolerance,
												stuckDebounceSeconds
											)
										)
										.asProxy(),
									new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_CLOSE_INTAKE_AFTER_PATH_END_SECONDS)
										.andThen(closeIntake.get())
								)
							)
					)
				)
			),
			new Pose2d(),
			"L quarter to outpost"
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getFlippedLQuarterAuto(
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
		return () -> new PathPlannerAutoWrapper(
			new ParallelCommandGroup(
				PathFollowingCommandsBuilder
					.followAdjustedPathThenStop(
						robot.getSwerve(),
						() -> robot.getPoseEstimator().getEstimatedPose(),
						PathHelper.PATH_PLANNER_PATHS.get("Flipped L quarter auto"),
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
						scoreSequence.get(),
						openIntake.get()
							.until(() -> hasPathEnded)
							.andThen(
								new ParallelCommandGroup(
									new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_START_WIGGLE_AFTER_PATH_END)
										.andThen(
											robot.getSwerve()
												.getCommandsBuilder()
												.wiggle(AutonomousConstants.WIGGLE_RANGE, AutonomousConstants.TIME_BETWEEN_WIGGLES_SECONDS)
												.withDeadline(new WaitCommand(AutonomousConstants.TIME_TO_WAIT_AT_DEPOT))
										)
										.asProxy(),
									new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_CLOSE_INTAKE_AFTER_PATH_END_SECONDS)
										.andThen(closeIntake.get())
								)
							)
					)
				)
			),
			new Pose2d(),
			"Flipped L quarter auto"
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getHorseshoeAuto(
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
		return () -> new PathPlannerAutoWrapper(
			new ParallelCommandGroup(
				PathFollowingCommandsBuilder
					.followAdjustedPathThenStop(
						robot.getSwerve(),
						() -> robot.getPoseEstimator().getEstimatedPose(),
						startingSide == AllianceSide.DEPOT
							? PathHelper.PATH_PLANNER_PATHS.get("L horseshoe")
							: PathHelper.PATH_PLANNER_PATHS.get("R horseshoe"),
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
												.withDeadline(new WaitCommand(AutonomousConstants.TIME_TO_WAIT_AT_DEPOT))
										)
										.andThen(
											getAllianceSideToStartingLineAuto(
												robot,
												startingSide.getOppositeSide(),
												pathfindingConstraints,
												regularIsNearEndOfPathTolerance,
												stuckIsNearEndOfPathTolerance,
												stuckDebounceSeconds
											)
										)
										.asProxy(),
									new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_CLOSE_INTAKE_AFTER_PATH_END_SECONDS)
										.andThen(closeIntake.get())
								)
							)
					)
				)
			),
			new Pose2d(),
			startingSide == AllianceSide.OUTPOST ? "R horseshoe" : "L horseshoe"
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getPushAuto(
		Robot robot,
		Supplier<Command> resetSubsystems,
		Supplier<Command> openIntake,
		Supplier<Command> closeIntake,
		Supplier<Command> scoreSequence,
		Supplier<Command> passSequence,
		Supplier<Command> outtakeSequence,
		PathConstraints pathfindingConstraints,
		Pose2d regularIsNearEndOfPathTolerance,
		Pose2d stuckIsNearEndOfPathTolerance,
		double stuckDebounceSeconds
	) {
		return () -> new PathPlannerAutoWrapper(
			new ParallelCommandGroup(
				PathFollowingCommandsBuilder
					.followAdjustedPathThenStop(
						robot.getSwerve(),
						() -> robot.getPoseEstimator().getEstimatedPose(),
						PathHelper.PATH_PLANNER_PATHS.get("Push Auto"),
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
						new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_START_PASSING_AFTER_PUSH_AUTO_START)
							.andThen(passSequence.get().withTimeout(AutonomousConstants.TIME_TO_PASS_IN_PUSH_AUTO))
							.andThen(scoreSequence.get()),
						((openIntake.get().withTimeout(10)).andThen(outtakeSequence.get().withTimeout(1.5)))
							.andThen(openIntake.get().withTimeout(4.2))
							.andThen(outtakeSequence.get().withTimeout(1.75))
							.andThen(openIntake.get())
							.until(() -> hasPathEnded)
							.andThen(
								new ParallelCommandGroup(
									robot.getSwerve()
										.getCommandsBuilder()
										.wiggle(AutonomousConstants.WIGGLE_RANGE, AutonomousConstants.TIME_BETWEEN_WIGGLES_SECONDS),
									new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_CLOSE_INTAKE_AFTER_PATH_END_SECONDS)
										.andThen(closeIntake.get())
								).asProxy()
							)
					)
				)
			),
			new Pose2d(),
			"Push Auto"
		);
	}

	private static Command getAllianceSideToStartingLineAuto(
		Robot robot,
		AllianceSide allianceSide,
		PathConstraints pathfindingConstraints,
		Pose2d regularIsNearEndOfPathTolerance,
		Pose2d stuckIsNearEndOfPathTolerance,
		double stuckDebounceSeconds
	) {
		return PathFollowingCommandsBuilder
			.followAdjustedPathThenStop(
				robot.getSwerve(),
				() -> robot.getPoseEstimator().getEstimatedPose(),
				allianceSide == AllianceSide.DEPOT
					? PathHelper.PATH_PLANNER_PATHS.get("Depot - Starting line")
					: PathHelper.PATH_PLANNER_PATHS.get("Outpost - Starting line"),
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				robot.getSwerve().getLogPath()
			)
			.andThen(
				robot.getSwerve().getCommandsBuilder().wiggle(AutonomousConstants.WIGGLE_RANGE, AutonomousConstants.TIME_BETWEEN_WIGGLES_SECONDS)
			);
	}

}
