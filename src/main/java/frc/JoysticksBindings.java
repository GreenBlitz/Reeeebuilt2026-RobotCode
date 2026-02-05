package frc;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.field.Field;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.autonomous.PathFollowingCommandsBuilder;
import frc.robot.statemachine.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.utils.auto.PathHelper;
import frc.utils.battery.BatteryUtil;
import frc.utils.math.ToleranceMath;

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
		} else {
			chassisDriverInputs.xPower = 0;
			chassisDriverInputs.yPower = 0;
			chassisDriverInputs.rotationalPower = 0;
		}
	}

	private static void mainJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = MAIN_JOYSTICK;
		// bindings...
		usedJoystick.Y.onTrue(robot.getRobotCommander().driveWith(RobotState.PRE_SCORE, robot.getRobotCommander().scoreSequence()));
		usedJoystick.A.onTrue(robot.getRobotCommander().driveWith(RobotState.NEUTRAL));

		usedJoystick.POV_LEFT.onTrue(robot.getRobotCommander().calibrationScoreSequence());
	}

	private static void secondJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SECOND_JOYSTICK;
		// bindings...
	}

	private static void thirdJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = THIRD_JOYSTICK;
		// bindings...
//		usedJoystick.B.onTrue(PathFollowingCommandsBuilder.pathfindThenFollowPath(PathHelper.PATH_PLANNER_PATHS.get("right to left"), new PathConstraints(2.0,2.0, 5, 5)).andThen(robot.getSwerve().getCommandsBuilder().drive(() -> new ChassisPowers())));
		usedJoystick.B.onTrue(robot.getSwerve().getCommandsBuilder().driveToPath(
				() -> robot.getPoseEstimator().getEstimatedPose(),
				PathHelper.PATH_PLANNER_PATHS.get("right to left"),
				new Pose2d(1.1, 7.1, Rotation2d.fromDegrees(51)),
				new PathConstraints(3, 3, 5, 5)
		));
		usedJoystick.Y.onTrue(PathFollowingCommandsBuilder.pathfindThenFollowPath(PathHelper.PATH_PLANNER_PATHS.get("left to right"), new PathConstraints(2.0,2.0, 5, 5)).andThen(robot.getSwerve().getCommandsBuilder().drive(() -> new ChassisPowers())));
//		usedJoystick.X.onTrue(PathFollowingCommandsBuilder.followPath(PathHelper.PATH_PLANNER_PATHS.get("RIGHT2LEFT")).andThen(robot.getSwerve().getCommandsBuilder().drive(() -> new ChassisPowers())));
		usedJoystick.A.onTrue(robot.getRobotCommander().driveWith(RobotState.NEUTRAL));
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

	private static void applyRobotCommanderCalibrationsBinding(SmartJoystick joystick, Robot robot) {
		joystick.A.onTrue(robot.getRobotCommander().driveWith(RobotState.NEUTRAL));
		joystick.B.onTrue(robot.getRobotCommander().driveWith(RobotState.PRE_SCORE));
		joystick.X.onTrue(robot.getRobotCommander().driveWith(RobotState.SCORE));
		joystick.POV_DOWN.onTrue(robot.getRobotCommander().driveWith(RobotState.STAY_IN_PLACE));
		joystick.POV_LEFT.onTrue(robot.getRobotCommander().calibrationScoreSequence());
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
