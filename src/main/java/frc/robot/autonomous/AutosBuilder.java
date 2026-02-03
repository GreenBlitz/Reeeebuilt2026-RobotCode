package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.statemachine.intakestatehandler.IntakeState;
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

		public static List<Supplier<PathPlannerAutoWrapper>> getRightFirstQuarterAuto(Robot robot) {
			return List.of(
					() -> new PathPlannerAutoWrapper(
//							new SequentialCommandGroup(
//									new ParallelDeadlineGroup(
						 			PathFollowingCommandsBuilder.followPath(PathHelper.PATH_PLANNER_PATHS.get("Right start 1-1")),
//											new RunCommand(() -> {}).until(() -> robot.getPoseEstimator().getEstimatedPose().getX()>6).andThen(robot.getRobotCommander().getIntakeStateHandler().setState(IntakeState.INTAKE)
//									)),
//									new ParallelDeadlineGroup(
//								PathFollowingCommandsBuilder.followPath(PathHelper.PATH_PLANNER_PATHS.get("Right 1-2")),
//						  robot.getRobotCommander().scoreSequence()
//									)
//							),
							new Pose2d(10,10,new Rotation2d()),
							"Start Of Autonomous"
					)
			);
		}


	}
