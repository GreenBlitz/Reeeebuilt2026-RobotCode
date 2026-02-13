package frc;

import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.autonomous.PathFollowingCommandsBuilder;
import frc.robot.statemachine.RobotState;
import frc.robot.statemachine.funnelstatehandler.FunnelState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.utils.auto.PathHelper;
import frc.utils.battery.BatteryUtil;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;


public class JoysticksBindings {

	private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(JoystickPorts.MAIN);
	private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(JoystickPorts.SECOND);
	private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(JoystickPorts.THIRD);
	private static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(JoystickPorts.FOURTH);
	private static final SmartJoystick FIFTH_JOYSTICK = new SmartJoystick(JoystickPorts.FIFTH);
	private static final SmartJoystick SIXTH_JOYSTICK = new SmartJoystick(JoystickPorts.SIXTH);

	private static final ChassisPowers chassisDriverInputs = new ChassisPowers();

	public static void configureBindings(Robot robot) {
		robot.getSwerve().setDriversPowerInputs(chassisDriverInputs);

		mainJoystickButtons(robot);
		secondJoystickButtons(robot);
		thirdJoystickButtons(robot);
		fourthJoystickButtons(robot);
		fifthJoystickButtons(robot);
		sixthJoystickButtons(robot);
	}

	public static void updateChassisDriverInputs() {
		if (MAIN_JOYSTICK.isConnected()) {
			chassisDriverInputs.xPower = MAIN_JOYSTICK.getAxisValue(Axis.LEFT_Y);
			chassisDriverInputs.yPower = MAIN_JOYSTICK.getAxisValue(Axis.LEFT_X);
			chassisDriverInputs.rotationalPower = MAIN_JOYSTICK.getAxisValue(Axis.RIGHT_X);
		} else if (THIRD_JOYSTICK.isConnected()) {
			chassisDriverInputs.xPower = THIRD_JOYSTICK.getAxisValue(Axis.LEFT_Y);
			chassisDriverInputs.yPower = THIRD_JOYSTICK.getAxisValue(Axis.LEFT_X);
			chassisDriverInputs.rotationalPower = THIRD_JOYSTICK.getAxisValue(Axis.RIGHT_X);
		} else if (SECOND_JOYSTICK.isConnected()) {
			chassisDriverInputs.xPower = SECOND_JOYSTICK.getAxisValue(Axis.LEFT_Y);
			chassisDriverInputs.yPower = SECOND_JOYSTICK.getAxisValue(Axis.LEFT_X);
			chassisDriverInputs.rotationalPower = SECOND_JOYSTICK.getAxisValue(Axis.RIGHT_X);
		} else {
			chassisDriverInputs.xPower = 0;
			chassisDriverInputs.yPower = 0;
			chassisDriverInputs.rotationalPower = 0;
		}
	}

	private static void mainJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = MAIN_JOYSTICK;
		// bindings...
		usedJoystick.A.onTrue(robot.getRobotCommander().driveWith(RobotState.NEUTRAL));
		usedJoystick.Y.onTrue(robot.getRobotCommander().driveWith(RobotState.PRE_SCORE, robot.getRobotCommander().scoreSequence()));
		usedJoystick.START.onTrue(new InstantCommand(() -> robot.getPoseEstimator().resetPose(robot.getPoseEstimator().getEstimatedPose())));
	}

	private static void secondJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SECOND_JOYSTICK;
		// bindings...
		applyShootOnMoveBinds(usedJoystick, robot);
	}

	private static void thirdJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = THIRD_JOYSTICK;
		// bindings...
		applyInterpolationCalibrationBindings(usedJoystick, robot);
	}

	private static void fourthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FOURTH_JOYSTICK;
		// bindings...
	}

	private static void fifthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FIFTH_JOYSTICK;
		// bindings...
	}

	private static void sixthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SIXTH_JOYSTICK;
		// bindings...
	}

	private static void applyInterpolationCalibrationBindings(SmartJoystick joystick, Robot robot) {
		joystick.A.onTrue(robot.getRobotCommander().driveWith(RobotState.NEUTRAL));
		joystick.Y.onTrue(robot.getRobotCommander().scoreSequence());
		joystick.POV_LEFT.onTrue(robot.getRobotCommander().calibrationScoreSequence());
	}

	private static void applyShootOnMoveBinds(SmartJoystick usedJoystick, Robot robot) {
		usedJoystick.A.onTrue(robot.getRobotCommander().driveWith(RobotState.NEUTRAL));
		usedJoystick.R1.onTrue(robot.getRobotCommander().scoreSequence());

		new EventTrigger("pre_shoot").onTrue(robot.getRobotCommander().getFunnelStateHandler().setState(FunnelState.NEUTRAL).asProxy());
		new EventTrigger("shoot").onTrue(robot.getRobotCommander().getFunnelStateHandler().setState(FunnelState.SHOOT).asProxy());

		PathPlannerPath depotToOutpost = PathHelper.PATH_PLANNER_PATHS.get("Depot-to-Outpost");
		usedJoystick.B.onTrue(
			new SequentialCommandGroup(
				PathFollowingCommandsBuilder
					.pathfindToPose(depotToOutpost.flipPath().getStartingHolonomicPose().get(), new PathConstraints(2, 2, 3, 3)),
				robot.getRobotCommander().setState(RobotState.PRE_SCORE).until(() -> robot.getRobotCommander().isReadyToScore()),
				new ParallelCommandGroup(
					PathFollowingCommandsBuilder.followPath(depotToOutpost)
						.alongWith(new InstantCommand(() -> Logger.recordOutput("StartedPath", TimeUtil.getCurrentTimeSeconds()))),
					robot.getRobotCommander().scoreSequence()
				)
			)
		);

		PathPlannerPath outpostToDepot = PathHelper.PATH_PLANNER_PATHS.get("Outpost-to-Depot");
		usedJoystick.X.onTrue(
			new SequentialCommandGroup(
				PathFollowingCommandsBuilder
					.pathfindToPose(outpostToDepot.flipPath().getStartingHolonomicPose().get(), new PathConstraints(2, 2, 3, 3)),
				robot.getRobotCommander().setState(RobotState.PRE_SCORE).until(() -> robot.getRobotCommander().isReadyToScore()),
				new ParallelCommandGroup(
					PathFollowingCommandsBuilder.followPath(outpostToDepot)
						.alongWith(new InstantCommand(() -> Logger.recordOutput("StartedPath", TimeUtil.getCurrentTimeSeconds()))),
					robot.getRobotCommander().scoreSequence()
				)
			)
		);

		PathPlannerPath towerToHub = PathHelper.PATH_PLANNER_PATHS.get("Tower-to-Hub");
		usedJoystick.POV_UP.onTrue(
				new SequentialCommandGroup(
						PathFollowingCommandsBuilder
								.pathfindToPose(towerToHub.flipPath().getStartingHolonomicPose().get(), new PathConstraints(2, 2, 3, 3)),
						robot.getRobotCommander().setState(RobotState.PRE_SCORE).until(() -> robot.getRobotCommander().isReadyToScore()),
						new ParallelCommandGroup(
								PathFollowingCommandsBuilder.followPath(towerToHub)
										.alongWith(new InstantCommand(() -> Logger.recordOutput("StartedPath", TimeUtil.getCurrentTimeSeconds()))),
								robot.getRobotCommander().scoreSequence()
						)
				)
		);

		PathPlannerPath hubToTower = PathHelper.PATH_PLANNER_PATHS.get("Hub-to-Tower");
		usedJoystick.POV_DOWN.onTrue(
				new SequentialCommandGroup(
						PathFollowingCommandsBuilder
								.pathfindToPose(hubToTower.flipPath().getStartingHolonomicPose().get(), new PathConstraints(2, 2, 3, 3)),
						robot.getRobotCommander().setState(RobotState.PRE_SCORE).until(() -> robot.getRobotCommander().isReadyToScore()),
						new ParallelCommandGroup(
								PathFollowingCommandsBuilder.followPath(hubToTower)
										.alongWith(new InstantCommand(() -> Logger.recordOutput("StartedPath", TimeUtil.getCurrentTimeSeconds()))),
								robot.getRobotCommander().scoreSequence()
						)
				)
		);

		PathPlannerPath rightToLeftDiagonal = PathHelper.PATH_PLANNER_PATHS.get("Right-To-Left-Diagonal");
		usedJoystick.POV_LEFT.onTrue(
				new SequentialCommandGroup(
						PathFollowingCommandsBuilder
								.pathfindToPose(rightToLeftDiagonal.flipPath().getStartingHolonomicPose().get(), new PathConstraints(2, 2, 3, 3)),
						robot.getRobotCommander().setState(RobotState.PRE_SCORE).until(() -> robot.getRobotCommander().isReadyToScore()),
						new ParallelCommandGroup(
								PathFollowingCommandsBuilder.followPath(rightToLeftDiagonal)
										.alongWith(new InstantCommand(() -> Logger.recordOutput("StartedPath", TimeUtil.getCurrentTimeSeconds()))),
								robot.getRobotCommander().scoreSequence()
						)
				)
		);

		PathPlannerPath leftToRightDiagonal = PathHelper.PATH_PLANNER_PATHS.get("Left-to-Right-Diagonal");
		usedJoystick.POV_RIGHT.onTrue(
				new SequentialCommandGroup(
						PathFollowingCommandsBuilder
								.pathfindToPose(leftToRightDiagonal.flipPath().getStartingHolonomicPose().get(), new PathConstraints(2, 2, 3, 3)),
						robot.getRobotCommander().setState(RobotState.PRE_SCORE).until(() -> robot.getRobotCommander().isReadyToScore()),
						new ParallelCommandGroup(
								PathFollowingCommandsBuilder.followPath(leftToRightDiagonal)
										.alongWith(new InstantCommand(() -> Logger.recordOutput("StartedPath", TimeUtil.getCurrentTimeSeconds()))),
								robot.getRobotCommander().scoreSequence()
						)
				)
		);


//		PathPlannerPath depotToOutpost2 = PathHelper.PATH_PLANNER_PATHS.get("Depot-to-Outpost");
//		usedJoystick.POV_RIGHT.onTrue(
//			new SequentialCommandGroup(
//				PathFollowingCommandsBuilder
//					.pathfindToPose(depotToOutpost2.flipPath().getStartingHolonomicPose().get(), new PathConstraints(2, 2, 3, 3))
//					.asProxy(),
//				new ParallelCommandGroup(
//					PathFollowingCommandsBuilder.followPath(depotToOutpost2)
//						.alongWith(new InstantCommand(() -> Logger.recordOutput("StartedPath", TimeUtil.getCurrentTimeSeconds())))
//						.asProxy()
//				)
//			)
//		);
//
//		PathPlannerPath outpostToDepot2 = PathHelper.PATH_PLANNER_PATHS.get("Outpost-to-Depot");
//		usedJoystick.POV_LEFT.onTrue(
//			new SequentialCommandGroup(
//				PathFollowingCommandsBuilder
//					.pathfindToPose(outpostToDepot2.flipPath().getStartingHolonomicPose().get(), new PathConstraints(2, 2, 3, 3))
//					.asProxy(),
//				new ParallelCommandGroup(
//					PathFollowingCommandsBuilder.followPath(outpostToDepot2)
//						.alongWith(new InstantCommand(() -> Logger.recordOutput("StartedPath", TimeUtil.getCurrentTimeSeconds())))
//						.asProxy()
//				)
//			)
//		);
	}

	private static void applyRobotCommanderCalibrationsBinding(SmartJoystick joystick, Robot robot) {
		joystick.A.onTrue(robot.getRobotCommander().driveWith(RobotState.NEUTRAL));
		joystick.B.onTrue(robot.getRobotCommander().driveWith(RobotState.PRE_SCORE));
		joystick.X.onTrue(robot.getRobotCommander().driveWith(RobotState.SCORE));
		joystick.POV_DOWN.onTrue(robot.getRobotCommander().driveWith(RobotState.STAY_IN_PLACE));
		joystick.POV_LEFT
			.onTrue(robot.getRobotCommander().driveWith(RobotState.CALIBRATION_PRE_SCORE, robot.getRobotCommander().calibrationScoreSequence()));
	}

	private static void applyTurretCalibrationBindings(Arm turret, SmartJoystick joystick, double calibrationMaxPower) {
		// Check limits
		joystick.R1.whileTrue(turret.getCommandsBuilder().setPower(() -> joystick.getAxisValue(Axis.LEFT_Y) * calibrationMaxPower));
		turret.getSysIdCalibrator().setAllButtonsForCalibration(joystick);

		joystick.POV_RIGHT.onTrue(turret.getCommandsBuilder().setTargetPosition(Rotation2d.fromDegrees(20)));
		joystick.POV_LEFT.onTrue(turret.getCommandsBuilder().setTargetPosition(Rotation2d.fromDegrees(50)));
	}

	private static void applyHoodCalibrationBindings(Arm hood, SmartJoystick joystick, double calibrationMaxPower) {
		// Check limits
		joystick.R1.whileTrue(
			hood.getCommandsBuilder()
				.setPower(
					() -> joystick.getAxisValue(Axis.LEFT_Y) * calibrationMaxPower + (hood.getKgVoltage() / BatteryUtil.getCurrentVoltage())
				)
		);

		hood.getSysIdCalibrator().setAllButtonsForCalibration(joystick);

		joystick.POV_RIGHT.onTrue(hood.getCommandsBuilder().setTargetPosition(Rotation2d.fromDegrees(57)));
		joystick.POV_LEFT.onTrue(hood.getCommandsBuilder().setTargetPosition(Rotation2d.fromDegrees(35)));
	}

	private static void applyTrainCalibrationBindings(Roller train, SmartJoystick joystick, double maxCalibrationPower) {
		joystick.X.onTrue(train.getCommandsBuilder().setPower(0.5 * maxCalibrationPower));
		joystick.Y.onTrue(train.getCommandsBuilder().setPower(-0.5 * maxCalibrationPower));
	}

	private static void applyBellyCalibrationBindings(Roller belly, SmartJoystick joystick, double maxCalibrationPower) {
		joystick.X.onTrue(belly.getCommandsBuilder().setPower(0.5 * maxCalibrationPower));
		joystick.Y.onTrue(belly.getCommandsBuilder().setPower(-0.5 * maxCalibrationPower));
	}

}
