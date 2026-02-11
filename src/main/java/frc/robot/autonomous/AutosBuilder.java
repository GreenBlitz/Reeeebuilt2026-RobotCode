package frc.robot.autonomous;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
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



	public static Command moveSidesThroughNeutralZone(
			Robot robot,
			Supplier<Command> intake,
			Supplier<Command> resetSubsystems,
			PathConstraints pathfindingConstraints,
			Pose2d tolerance,
			boolean isLeft
	){
		return PathFollowingCommandsBuilder.deadlineCommandWithPath(
				robot.getSwerve(),
				() -> robot.getPoseEstimator().getEstimatedPose(),
				isLeft
						? PathHelper.PATH_PLANNER_PATHS.get("R starting - Left finish through neutral zone").mirrorPath()
						: PathHelper.PATH_PLANNER_PATHS.get("R starting - Left finish through neutral zone"),
				pathfindingConstraints,
				() -> resetSubsystems.get().andThen(intake.get()),
				tolerance
		);
	}
}
