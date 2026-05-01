package frc.robot.autonomous;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.constants.field.AllianceSide;
import frc.robot.Robot;
import frc.utils.auto.PathHelper;
import frc.utils.auto.PathPlannerAutoWrapper;

import java.util.List;
import java.util.function.BooleanSupplier;
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
		Supplier<Command> passSequence,
		Supplier<Command> outtakeSequence,
		PathConstraints pathfindingConstraints,
		Pose2d regularIsNearEndOfPathTolerance,
		Pose2d stuckIsNearEndOfPathTolerance,
		double stuckDebounceSeconds,
		BooleanSupplier returnToMiddle
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
				AllianceSide.DEPOT,
				returnToMiddle
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
				AllianceSide.OUTPOST,
				returnToMiddle
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
				AllianceSide.DEPOT,
				false,
				returnToMiddle
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
				AllianceSide.DEPOT,
				true,
				returnToMiddle
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
				AllianceSide.OUTPOST,
				true,
				returnToMiddle
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
				stuckDebounceSeconds,
				returnToMiddle
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
				AllianceSide.DEPOT,
				returnToMiddle
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
				AllianceSide.OUTPOST,
				returnToMiddle
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
			),
			getMiddleAuto(
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
			getStealAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.DEPOT,
				AllianceSide.DEPOT,
				AllianceSide.DEPOT,
				false,
				returnToMiddle
			),
			getStealAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.OUTPOST,
				AllianceSide.DEPOT,
				AllianceSide.DEPOT,
				false,
				returnToMiddle
			),
			getStealAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.DEPOT,
				AllianceSide.DEPOT,
				AllianceSide.OUTPOST,
				false,
				returnToMiddle
			),
			getStealAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.DEPOT,
				AllianceSide.OUTPOST,
				AllianceSide.DEPOT,
				false,
				returnToMiddle
			),
			getStealAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.OUTPOST,
				AllianceSide.OUTPOST,
				AllianceSide.DEPOT,
				false,
				returnToMiddle
			),
			getStealAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.DEPOT,
				AllianceSide.OUTPOST,
				AllianceSide.OUTPOST,
				false,
				returnToMiddle
			),
			getStealAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.OUTPOST,
				AllianceSide.DEPOT,
				AllianceSide.OUTPOST,
				false,
				returnToMiddle
			),
			getStealAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.OUTPOST,
				AllianceSide.OUTPOST,
				AllianceSide.OUTPOST,
				false,
				returnToMiddle
			),
			getStealAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.DEPOT,
				AllianceSide.OUTPOST,
				AllianceSide.OUTPOST,
				true,
				returnToMiddle
			),
			getStealAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.OUTPOST,
				AllianceSide.OUTPOST,
				AllianceSide.OUTPOST,
				true,
				returnToMiddle
			),
			getStealAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.DEPOT,
				AllianceSide.OUTPOST,
				AllianceSide.DEPOT,
				true,
				returnToMiddle
			),
			getStealAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.OUTPOST,
				AllianceSide.OUTPOST,
				AllianceSide.DEPOT,
				true,
				returnToMiddle
			),
			getLightStealAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.DEPOT,
				AllianceSide.DEPOT,
				returnToMiddle
			),
			getLightStealAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.DEPOT,
				AllianceSide.OUTPOST,
				returnToMiddle
			),
			getLightStealAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.OUTPOST,
				AllianceSide.OUTPOST,
				returnToMiddle
			),
			getLightStealAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.OUTPOST,
				AllianceSide.DEPOT,
				returnToMiddle
			),
			getSideAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.DEPOT,
				false,
				returnToMiddle
			),
			getSideAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.OUTPOST,
				false,
				returnToMiddle
			),
			getSideAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				AllianceSide.OUTPOST,
				true,
				returnToMiddle
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
		AllianceSide startingSide,
		BooleanSupplier returnToMiddle
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
												startingSide,
												pathfindingConstraints,
												regularIsNearEndOfPathTolerance,
												stuckIsNearEndOfPathTolerance,
												stuckDebounceSeconds,
												returnToMiddle
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
			startingSide == AllianceSide.DEPOT ? "L quarter" : "R quarter",
			startingSide == AllianceSide.DEPOT ? PathHelper.PATH_PLANNER_PATHS.get("L quarter") : PathHelper.PATH_PLANNER_PATHS.get("R quarter"),
			getAllianceSideToStartingLinePath(startingSide, returnToMiddle)
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getSideAuto(
		Robot robot,
		Supplier<Command> resetSubsystems,
		Supplier<Command> openIntake,
		Supplier<Command> closeIntake,
		Supplier<Command> scoreSequence,
		PathConstraints pathfindingConstraints,
		Pose2d regularIsNearEndOfPathTolerance,
		Pose2d stuckIsNearEndOfPathTolerance,
		double stuckDebounceSeconds,
		AllianceSide startingSide,
		boolean skipOutpost,
		BooleanSupplier returnToMiddle
	) {
		PathPlannerPath returnPath = startingSide == AllianceSide.DEPOT ? PathHelper.PATH_PLANNER_PATHS.get("Depot Side Steal")
			: skipOutpost ? PathHelper.PATH_PLANNER_PATHS.get("Outpost Side Steal to depot")
			: PathHelper.PATH_PLANNER_PATHS.get("Outpost Side Steal");

		return () -> new PathPlannerAutoWrapper(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					new ParallelDeadlineGroup(
						new WaitCommand(AutonomousConstants.SIDE_STEAL_START_SECOND_SINCE_AUTO_BEGAN),
						PathFollowingCommandsBuilder
							.followAdjustedPathThenStop(
								robot.getSwerve(),
								() -> robot.getPoseEstimator().getEstimatedPose(),
								startingSide == AllianceSide.DEPOT
									? PathHelper.PATH_PLANNER_PATHS.get("Depot Side Wait")
									: PathHelper.PATH_PLANNER_PATHS.get("Outpost Side Wait"),
								pathfindingConstraints,
								regularIsNearEndOfPathTolerance,
								stuckIsNearEndOfPathTolerance,
								stuckDebounceSeconds,
								robot.getSwerve().getLogPath()
							)
							.asProxy(),
						new InstantCommand(() -> hasPathEnded = false)
					),
					PathFollowingCommandsBuilder
						.followAdjustedPathThenStop(
							robot.getSwerve(),
							() -> robot.getPoseEstimator().getEstimatedPose(),
							returnPath,
							pathfindingConstraints,
							regularIsNearEndOfPathTolerance,
							stuckIsNearEndOfPathTolerance,
							stuckDebounceSeconds,
							robot.getSwerve().getLogPath()
						)
						.asProxy()
						.andThen(new InstantCommand(() -> hasPathEnded = true))
				),
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
												stuckDebounceSeconds,
												returnToMiddle
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
			"Side Steal: " + startingSide + ", Skip Outpost: " + skipOutpost,
			startingSide == AllianceSide.DEPOT
				? PathHelper.PATH_PLANNER_PATHS.get("Depot Side Wait")
				: PathHelper.PATH_PLANNER_PATHS.get("Outpost Side Wait"),
			startingSide == AllianceSide.DEPOT
				? PathHelper.PATH_PLANNER_PATHS.get("Depot Side Steal")
				: PathHelper.PATH_PLANNER_PATHS.get("Outpost Side Steal")
		);
	}

	private static PathPlannerPath getStealPath(AllianceSide firstOpponentBumpSide, AllianceSide returnSide, boolean skipOutpost) {
		String pathName;
		if (returnSide == AllianceSide.DEPOT) {
			pathName = firstOpponentBumpSide == AllianceSide.DEPOT ? "Depot Steal Cross" : "Depot Steal";
		} else {
			if (skipOutpost) {
				pathName = firstOpponentBumpSide == AllianceSide.DEPOT ? "Outpost Steal to Depot" : "Outpost Steal Cross to Depot";
			} else {
				pathName = firstOpponentBumpSide == AllianceSide.DEPOT ? "Outpost Steal" : "Outpost Steal Cross";
			}
		}

		return PathHelper.PATH_PLANNER_PATHS.get(pathName);
	}

	private static Supplier<PathPlannerAutoWrapper> getStealAuto(
		Robot robot,
		Supplier<Command> resetSubsystems,
		Supplier<Command> openIntake,
		Supplier<Command> closeIntake,
		Supplier<Command> scoreSequence,
		PathConstraints pathfindingConstraints,
		Pose2d regularIsNearEndOfPathTolerance,
		Pose2d stuckIsNearEndOfPathTolerance,
		double stuckDebounceSeconds,
		AllianceSide firstOpponentBumpSide,
		AllianceSide returnSide,
		AllianceSide startingSide,
		boolean skipOutpost,
		BooleanSupplier returnToMiddle
	) {
		AllianceSide actualReturnSide = skipOutpost || returnSide == AllianceSide.DEPOT ? AllianceSide.DEPOT : AllianceSide.OUTPOST;
		return () -> new PathPlannerAutoWrapper(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					new ParallelDeadlineGroup(
						new WaitCommand(AutonomousConstants.STEAL_START_SECOND_SINCE_AUTO_BEGAN),
						PathFollowingCommandsBuilder
							.followAdjustedPathThenStop(
								robot.getSwerve(),
								() -> robot.getPoseEstimator().getEstimatedPose(),
								startingSide == AllianceSide.DEPOT
									? PathHelper.PATH_PLANNER_PATHS.get("Depot Hub Wait")
									: PathHelper.PATH_PLANNER_PATHS.get("Outpost Hub Wait"),
								pathfindingConstraints,
								regularIsNearEndOfPathTolerance,
								stuckIsNearEndOfPathTolerance,
								stuckDebounceSeconds,
								robot.getSwerve().getLogPath()
							)
							.asProxy(),
						new InstantCommand(() -> hasPathEnded = false)
					),
					PathFollowingCommandsBuilder
						.followAdjustedPathThenStop(
							robot.getSwerve(),
							() -> robot.getPoseEstimator().getEstimatedPose(),
							getStealPath(firstOpponentBumpSide, returnSide, skipOutpost),
							pathfindingConstraints,
							regularIsNearEndOfPathTolerance,
							stuckIsNearEndOfPathTolerance,
							stuckDebounceSeconds,
							robot.getSwerve().getLogPath()
						)
						.asProxy()
						.andThen(new InstantCommand(() -> hasPathEnded = true))
				),
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
												actualReturnSide,
												pathfindingConstraints,
												regularIsNearEndOfPathTolerance,
												stuckIsNearEndOfPathTolerance,
												stuckDebounceSeconds,
												returnToMiddle
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
			"Steal start:" + startingSide + " Bump: " + firstOpponentBumpSide + " Return: " + returnSide + " Skip outpost: " + skipOutpost,
			startingSide == AllianceSide.DEPOT
				? PathHelper.PATH_PLANNER_PATHS.get("Depot Hub Wait")
				: PathHelper.PATH_PLANNER_PATHS.get("Outpost Hub Wait"),
			getStealPath(firstOpponentBumpSide, returnSide, skipOutpost),
			getAllianceSideToStartingLinePath(actualReturnSide, returnToMiddle)
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getLightStealAuto(
		Robot robot,
		Supplier<Command> resetSubsystems,
		Supplier<Command> openIntake,
		Supplier<Command> closeIntake,
		Supplier<Command> scoreSequence,
		PathConstraints pathfindingConstraints,
		Pose2d regularIsNearEndOfPathTolerance,
		Pose2d stuckIsNearEndOfPathTolerance,
		double stuckDebounceSeconds,
		AllianceSide startingSide,
		AllianceSide returnSide,
		BooleanSupplier returnToMiddle
	) {
		return () -> new PathPlannerAutoWrapper(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					new ParallelDeadlineGroup(
						new WaitCommand(AutonomousConstants.STEAL_START_SECOND_SINCE_AUTO_BEGAN),
						PathFollowingCommandsBuilder
							.followAdjustedPathThenStop(
								robot.getSwerve(),
								() -> robot.getPoseEstimator().getEstimatedPose(),
								startingSide == AllianceSide.DEPOT
									? PathHelper.PATH_PLANNER_PATHS.get("Depot Hub Wait")
									: PathHelper.PATH_PLANNER_PATHS.get("Outpost Hub Wait"),
								pathfindingConstraints,
								regularIsNearEndOfPathTolerance,
								stuckIsNearEndOfPathTolerance,
								stuckDebounceSeconds,
								robot.getSwerve().getLogPath()
							)
							.asProxy(),
						new InstantCommand(() -> hasPathEnded = false)
					),
					PathFollowingCommandsBuilder
						.followAdjustedPathThenStop(
							robot.getSwerve(),
							() -> robot.getPoseEstimator().getEstimatedPose(),
							returnSide == AllianceSide.DEPOT
								? PathHelper.PATH_PLANNER_PATHS.get("Light depot steal")
								: PathHelper.PATH_PLANNER_PATHS.get("Light outpost steal"),
							pathfindingConstraints,
							regularIsNearEndOfPathTolerance,
							stuckIsNearEndOfPathTolerance,
							stuckDebounceSeconds,
							robot.getSwerve().getLogPath()
						)
						.asProxy()
						.andThen(new InstantCommand(() -> hasPathEnded = true))
				),
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
												returnSide,
												pathfindingConstraints,
												regularIsNearEndOfPathTolerance,
												stuckIsNearEndOfPathTolerance,
												stuckDebounceSeconds,
												returnToMiddle
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
			"Light steal start:" + startingSide + " Return: " + returnSide,
			startingSide == AllianceSide.DEPOT
				? PathHelper.PATH_PLANNER_PATHS.get("Depot Hub Wait")
				: PathHelper.PATH_PLANNER_PATHS.get("Outpost Hub Wait"),
			returnSide == AllianceSide.DEPOT
				? PathHelper.PATH_PLANNER_PATHS.get("Light depot steal")
				: PathHelper.PATH_PLANNER_PATHS.get("Light outpost steal")
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
		AllianceSide startingSide,
		boolean endInOutpost,
		BooleanSupplier returnToMiddle
	) {
		return () -> new PathPlannerAutoWrapper(
			new ParallelCommandGroup(
				PathFollowingCommandsBuilder
					.followAdjustedPathThenStop(
						robot.getSwerve(),
						() -> robot.getPoseEstimator().getEstimatedPose(),
						startingSide == AllianceSide.DEPOT
							? endInOutpost
								? PathHelper.PATH_PLANNER_PATHS.get("L quarter light to outpost")
								: PathHelper.PATH_PLANNER_PATHS.get("L quarter light")
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
												.withDeadline(new WaitCommand(AutonomousConstants.TIME_TO_WIGGLE_IN_R_QUARTER_LIGHT))
										)
										.andThen(
											!returnToMiddle.getAsBoolean() && startingSide == AllianceSide.OUTPOST
												? robot.getSwerve()
													.getCommandsBuilder()
													.wiggle(AutonomousConstants.WIGGLE_RANGE, AutonomousConstants.TIME_BETWEEN_WIGGLES_SECONDS)
												: getAllianceSideToStartingLineAuto(
													robot,
													startingSide,
													pathfindingConstraints,
													regularIsNearEndOfPathTolerance,
													stuckIsNearEndOfPathTolerance,
													stuckDebounceSeconds,
													returnToMiddle
												)
										)
										.asProxy(),
									new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_CLOSE_INTAKE_AFTER_PATH_END_SECONDS)
										.andThen(closeIntake.get().onlyIf(() -> !returnToMiddle.getAsBoolean()))
								)
							)
					)
				)
			),
			new Pose2d(),
			startingSide == AllianceSide.DEPOT ? endInOutpost ? "L quarter light to outpost" : "L quarter light" : "R quarter light",
			startingSide == AllianceSide.DEPOT
				? endInOutpost
					? PathHelper.PATH_PLANNER_PATHS.get("L quarter light to outpost")
					: PathHelper.PATH_PLANNER_PATHS.get("L quarter light")
				: PathHelper.PATH_PLANNER_PATHS.get("R quarter light"),
			getAllianceSideToStartingLinePath(startingSide, returnToMiddle)
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
		double stuckDebounceSeconds,
		BooleanSupplier returnToMiddle
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
												stuckDebounceSeconds,
												returnToMiddle
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
			"L quarter to outpost",
			PathHelper.PATH_PLANNER_PATHS.get("L quarter to outpost"),
			PathHelper.PATH_PLANNER_PATHS.get("Outpost - Starting line")
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
		AllianceSide startingSide,
		BooleanSupplier returnToMiddle
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
												stuckDebounceSeconds,
												returnToMiddle
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
			startingSide == AllianceSide.OUTPOST ? "R horseshoe" : "L horseshoe",
			startingSide == AllianceSide.DEPOT
				? PathHelper.PATH_PLANNER_PATHS.get("L horseshoe")
				: PathHelper.PATH_PLANNER_PATHS.get("R horseshoe"),
			getAllianceSideToStartingLinePath(startingSide, returnToMiddle)
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
						((openIntake.get().withTimeout(AutonomousConstants.TIME_FOR_FIRST_INTAKE_OPEN_IN_PUSH_AUTO))
							.andThen(outtakeSequence.get().withTimeout(AutonomousConstants.TIME_FOR_FIRST_OUTTAKE_IN_PUSH_AUTO)))
							.andThen(openIntake.get().withTimeout(AutonomousConstants.TIME_FOR_SECOND_INTAKE_OPEN_IN_PUSH_AUTO))
							.andThen(outtakeSequence.get().withTimeout(AutonomousConstants.TIME_FOR_SECOND_OUTTAKE_IN_PUSH_AUTO))
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
			"Push Auto",
			PathHelper.PATH_PLANNER_PATHS.get("Push Auto")
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getMiddleAuto(
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
						PathHelper.PATH_PLANNER_PATHS.get("Middle path"),
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
									new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_START_WIGGLE_AFTER_PATH_END).andThen(
										robot.getSwerve()
											.getCommandsBuilder()
											.wiggle(AutonomousConstants.WIGGLE_RANGE, AutonomousConstants.TIME_BETWEEN_WIGGLES_SECONDS)
									),
									new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_CLOSE_INTAKE_AFTER_PATH_END_SECONDS)
										.andThen(closeIntake.get())
								).asProxy()
							)
					)
				)
			),
			new Pose2d(),
			"Middle Auto",
			PathHelper.PATH_PLANNER_PATHS.get("Middle path")
		);
	}

	private static Command getAllianceSideToStartingLineAuto(
		Robot robot,
		AllianceSide allianceSide,
		PathConstraints pathfindingConstraints,
		Pose2d regularIsNearEndOfPathTolerance,
		Pose2d stuckIsNearEndOfPathTolerance,
		double stuckDebounceSeconds,
		BooleanSupplier returnToMiddle
	) {
		return PathFollowingCommandsBuilder
			.followAdjustedPathThenStop(
				robot.getSwerve(),
				() -> robot.getPoseEstimator().getEstimatedPose(),
				getAllianceSideToStartingLinePath(allianceSide, returnToMiddle),
				pathfindingConstraints,
				regularIsNearEndOfPathTolerance,
				stuckIsNearEndOfPathTolerance,
				stuckDebounceSeconds,
				robot.getSwerve().getLogPath()
			)
			.andThen(
				robot.getSwerve()
					.getCommandsBuilder()
					.wiggle(AutonomousConstants.WIGGLE_RANGE, AutonomousConstants.TIME_BETWEEN_WIGGLES_SECONDS)
					.onlyIf(() -> !returnToMiddle.getAsBoolean())
			);
	}

	private static PathPlannerPath getAllianceSideToStartingLinePath(AllianceSide allianceSide, BooleanSupplier returnToMiddle) {
		if (returnToMiddle.getAsBoolean()) {
			return allianceSide == AllianceSide.DEPOT
				? PathHelper.PATH_PLANNER_PATHS.get("Depot - Middle")
				: PathHelper.PATH_PLANNER_PATHS.get("Outpost - Middle");
		}
		return allianceSide == AllianceSide.DEPOT
			? PathHelper.PATH_PLANNER_PATHS.get("Depot - Starting line")
			: PathHelper.PATH_PLANNER_PATHS.get("Outpost - Starting line");
	}

}
