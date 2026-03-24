// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.revrobotics.util.StatusLogger;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.utils.GamePeriodUtils;
import frc.utils.HubUtil;
import frc.utils.brakestate.BrakeMode;
import frc.utils.driverstation.DriverStationUtil;
import frc.utils.alerts.AlertManager;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.brakestate.BrakeStateManager;
import frc.utils.logger.LoggerFactory;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the build.gradle file in
 * the project.
 */
public class RobotManager extends LoggedRobot {

	private final Robot robot;
	private final TimeInterpolatableBuffer<Double> ballsBuffer;
	private PathPlannerAutoWrapper autonomousCommand;
	private int roborioCycles;
	private static double teleopStartTimeSeconds = -1;
	private static double autonomousStartTimeSeconds = -1;
	private static double ballCounter;

	public RobotManager() {
		StatusLogger.disableAutoLogging();
		if (Robot.ROBOT_TYPE.isReplay()) {
			setUseTiming(false);
		}
		DriverStation.silenceJoystickConnectionWarning(true);
		LoggerFactory.initializeLogger();
		PathPlannerUtil.startPathfinder();
		PathPlannerUtil.setupPathPlannerLogging();

		this.roborioCycles = 0;
		this.robot = new Robot();

		ballsBuffer = TimeInterpolatableBuffer.createBuffer(Interpolator.forDouble(), RobotConstants.MAX_TIME_FOR_BPS_INTERPOLATOR);
		new Trigger(() -> robot.getRobotCommander().getShooterStateHandler().hasABallBeenShot()).onTrue(new InstantCommand(() -> {
			ballCounter++;
			ballsBuffer.addSample(TimeUtil.getCurrentTimeSeconds(), ballCounter);
		}));

		new Trigger(GamePeriodUtils::isTransitionShift).onFalse(new InstantCommand(() -> Logger.recordOutput("averagePeriodPBS/TransitionShift", getAverageBPSForLastXSeconds(GamePeriodUtils.AUTONOMOUS_DURATION_SECONDS))));
		new Trigger(GamePeriodUtils::isInEndgame).onFalse(new InstantCommand(() -> Logger.recordOutput("averagePeriodPBS/Endgame", getAverageBPSForLastXSeconds(GamePeriodUtils.AUTONOMOUS_DURATION_SECONDS))));

		JoysticksBindings.configureBindings(robot);

		Threads.setCurrentThreadPriority(true, 10);

		CommandScheduler.getInstance()
			.schedule(
				new WaitCommand(1).andThen(new InstantCommand(() -> robot.getAutonomousChooser().getChooser().onChange((autonomousCommand) -> {
					this.autonomousCommand = autonomousCommand.get();
				})))
			);
	}

	@Override
	public void disabledInit() {
		if (!DriverStationUtil.isMatch()) {
			BrakeStateManager.setBrakeMode(BrakeMode.COAST);
		}
	}

	@Override
	public void disabledExit() {
		if (!DriverStationUtil.isMatch()) {
			BrakeStateManager.setBrakeMode(BrakeMode.BRAKE);
		}
	}

	@Override
	public void autonomousInit() {
		autonomousStartTimeSeconds = TimeUtil.getCurrentTimeSeconds();
		ballCounter = 0;
		robot.getSwerve().setIsRunningIndependently(true);

		if (autonomousCommand == null) {
			this.autonomousCommand = robot.getAutonomousChooser().getChosenValue();
		}
		CommandScheduler.getInstance().schedule(autonomousCommand);
	}

	@Override
	public void autonomousExit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}

		robot.getSwerve().setIsRunningIndependently(false);
		Logger.recordOutput("averagePeriodPBS/Autonomous", getAverageBPSForLastXSeconds(GamePeriodUtils.AUTONOMOUS_DURATION_SECONDS));
	}

	@Override
	public void teleopInit() {
		teleopStartTimeSeconds = TimeUtil.getCurrentTimeSeconds();
	}

	@Override
	public void teleopExit() {
		robot.getLimelightFront().captureGivenTime(GamePeriodUtils.COMPLETE_GAME_TIME_SECONDS);
		robot.getLimelightLeft().captureGivenTime(GamePeriodUtils.COMPLETE_GAME_TIME_SECONDS);
		robot.getLimelightRight().captureGivenTime(GamePeriodUtils.COMPLETE_GAME_TIME_SECONDS);
	}

	public static double getTeleopStartTimeSeconds() {
		return teleopStartTimeSeconds;
	}

	public static double getAutonomousStartTimeSeconds() {
		return autonomousStartTimeSeconds;
	}

	@Override
	public void simulationPeriodic() {
		robot.getSimulationManager().logPoses();
	}

	@Override
	public void robotPeriodic() {
		updateTimeRelatedData(); // Better to be first
		JoysticksBindings.updateChassisDriverInputs();
		HubUtil.refreshAlliances();
		robot.periodic();
		AlertManager.reportAlerts();

		Logger.recordOutput("BallCounter", ballCounter);
		Logger.recordOutput("CurrentBPS", getAverageBPSForLastXSeconds(RobotConstants.TIME_FOR_AVERAGE_BPS_CALCULATION_SECONDS));
	}

	private void updateTimeRelatedData() {
		roborioCycles++;
		Logger.recordOutput("RoborioCycles", roborioCycles);
		TimeUtil.updateCycleTime(roborioCycles);
	}

	private double getAverageBPSForLastXSeconds(double seconds){
		if (ballsBuffer.getSample(TimeUtil.getCurrentTimeSeconds() - seconds).isPresent()) {
			return (ballCounter - ballsBuffer.getSample(TimeUtil.getCurrentTimeSeconds() - seconds).get()) / seconds;
		}
		return 0;
	}

}
