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
			new SequentialCommandGroup(
				resetSubsystems.get(),
				new ParallelCommandGroup(
					scoreSequence.get(),
					new SequentialCommandGroup(
						PathFollowingCommandsBuilder.deadlineCommandWithPath(
							robot.getSwerve(),
							() -> robot.getPoseEstimator().getEstimatedPose(),
							startingSide == AllianceSide.DEPOT
								? PathHelper.PATH_PLANNER_PATHS.get("L quarter")
								: PathHelper.PATH_PLANNER_PATHS.get("R quarter"),
							pathfindingConstraints,
							openIntake,
							isNearEndOfPathTolerance,
							robot.getSwerve().getLogPath()
						),
						new WaitCommand(AutonomousConstants.TIME_TO_WAIT_TO_CLOSE_INTAKE_AFTER_ARRIVING_AT_FEEDER_SECONDS),
						closeIntake.get()
					)
				)
			),
			new Pose2d(),
			startingSide == AllianceSide.OUTPOST ? "R quarter" : "L quarter"
		);
	}


}
