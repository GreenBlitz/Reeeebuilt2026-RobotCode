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
		Pose2d isNearEndOfPathTolerance
	) {
		return List.of(
			getHorseshoeAuto(robot, intake, scoreSequence, resetSubsystems, pathfindingConstraints, isNearEndOfPathTolerance, AllianceSide.OUTPOST),
			getHorseshoeAuto(robot, intake, scoreSequence, resetSubsystems, pathfindingConstraints, isNearEndOfPathTolerance, AllianceSide.DEPOT),
			getQuarterAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				isNearEndOfPathTolerance,
				AllianceSide.OUTPOST
			),
			getQuarterAuto(
				robot,
				resetSubsystems,
				openIntake,
				closeIntake,
				scoreSequence,
				pathfindingConstraints,
				isNearEndOfPathTolerance,
				AllianceSide.DEPOT
			)
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getHorseshoeAuto(
		Robot robot,
		Supplier<Command> intake,
		Supplier<Command> scoreSequence,
		Supplier<Command> resetSubsystems,
		PathConstraints pathfindingConstraints,
		Pose2d isNearEndOfPathTolerance,
		AllianceSide allianceSide
	) {
		return () -> new PathPlannerAutoWrapper(
			new SequentialCommandGroup(
				startingLineToNeutralZoneCenterCommand(
					robot,
					intake,
					resetSubsystems,
					pathfindingConstraints,
					isNearEndOfPathTolerance,
					allianceSide
				),
				neutralZoneMiddleToStartingLineCommand(
					robot,
					intake,
					resetSubsystems,
					scoreSequence,
					pathfindingConstraints,
					isNearEndOfPathTolerance,
					allianceSide.getOtherSide()
				),
				startingLineToAllianceSideCommand(
						robot,
						intake,
						resetSubsystems,
						scoreSequence,
						pathfindingConstraints,
						isNearEndOfPathTolerance,
						allianceSide.getOtherSide()
					)
				),
			new Pose2d(),
			allianceSide == AllianceSide.OUTPOST ? "R starting - Neutral zone - Depot" : "L starting - Neutral zone - Outpost"
		);
	}

	private static Command startingLineToNeutralZoneCenterCommand(
		Robot robot,
		Supplier<Command> intake,
		Supplier<Command> resetSubsystems,
		PathConstraints pathfindingConstraints,
		Pose2d isNearEndOfPathTolerance,
		AllianceSide allianceSide
	) {
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
			robot.getSwerve(),
			() -> robot.getPoseEstimator().getEstimatedPose(),
			allianceSide == AllianceSide.OUTPOST
				? PathHelper.PATH_PLANNER_PATHS.get("R starting - Neutral zone center")
				: PathHelper.PATH_PLANNER_PATHS.get("R starting - Neutral zone center").mirrorPath(),
			pathfindingConstraints,
			() -> resetSubsystems.get().andThen(intake.get()),
			isNearEndOfPathTolerance,
			robot.getSwerve().getLogPath()
		);
	}
	
	private static Command startingLineToAllianceSideCommand(
			Robot robot,
			Supplier<Command> intake,
			Supplier<Command> resetSubsystems,
			Supplier<Command> scoreSequence,
			PathConstraints pathfindingConstraints,
			Pose2d isNearEndOfPathTolerance,
			AllianceSide allianceSide
	) {
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
				robot.getSwerve(),
				() -> robot.getPoseEstimator().getEstimatedPose(),
				allianceSide == AllianceSide.OUTPOST
						?PathHelper.PATH_PLANNER_PATHS.get("R starting - Outpost")
						:PathHelper.PATH_PLANNER_PATHS.get("L starting - Depot"),
				pathfindingConstraints,
				allianceSide == AllianceSide.OUTPOST
						?() -> resetSubsystems.get().andThen(scoreSequence.get())
						:() -> resetSubsystems.get().andThen(new ParallelCommandGroup(scoreSequence.get(),intake.get())),
				isNearEndOfPathTolerance,
				robot.getSwerve().getLogPath()
		);
	}
	
	private static Command neutralZoneMiddleToStartingLineCommand(
		Robot robot,
		Supplier<Command> intake,
		Supplier<Command> resetSubsystems,
		Supplier<Command> scoreSequence,
		PathConstraints pathfindingConstraints,
		Pose2d isNearEndOfPathTolerance,
		AllianceSide allianceSide
	) {
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
			robot.getSwerve(),
			() -> robot.getPoseEstimator().getEstimatedPose(),
			allianceSide == AllianceSide.OUTPOST
				? PathHelper.PATH_PLANNER_PATHS.get("Neutral zone center - Left starting line").mirrorPath()
				: PathHelper.PATH_PLANNER_PATHS.get("Neutral zone center - Left starting line"),
			pathfindingConstraints,
			() -> resetSubsystems.get().andThen(new ParallelCommandGroup(intake.get(), scoreSequence.get())),
			isNearEndOfPathTolerance,
			robot.getSwerve().getLogPath()
		);
	}

	private static Supplier<PathPlannerAutoWrapper> getQuarterAuto(
		Robot robot,
		Supplier<Command> resetSubsystems,
		Supplier<Command> openIntake,
		Supplier<Command> closeIntake,
		Supplier<Command> scoreSequence,
		PathConstraints pathfindingConstraints,
		Pose2d isNearEndOfPathTolerance,
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
						isNearEndOfPathTolerance,
						robot.getSwerve().getLogPath()

					)
					.andThen(new InstantCommand(() -> hasPathEnded = true)),
				new SequentialCommandGroup(
					resetSubsystems.get(),
					new ParallelCommandGroup(
						new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_START_SHOOTING_AFTER_AUTO_START).andThen(scoreSequence.get()),
						openIntake.get()
							.until(() -> hasPathEnded)
							.andThen(
								new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_CLOSE_INTAKE_AFTER_PATH_END_SECONDS)
									.andThen(closeIntake.get())
							)
					)
				)
			),
			new Pose2d(),
			startingSide == AllianceSide.OUTPOST ? "R quarter" : "L quarter"
		);
	}


}
