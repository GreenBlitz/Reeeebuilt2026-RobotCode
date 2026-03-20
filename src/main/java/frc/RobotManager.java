// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.revrobotics.util.StatusLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.subsystems.constants.flywheel.FlywheelConstants;
import frc.utils.GamePeriodUtils;
import frc.utils.HubUtil;
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
	private PathPlannerAutoWrapper autonomousCommand;
	private int roborioCycles;
	private static double teleopStartTimeSeconds = -1;
	private static double autonomousStartTimeSeconds = -1;
	private static double ballCounter;
	private static double lastBallShotTimestamp;

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

		JoysticksBindings.configureBindings(robot);

		Threads.setCurrentThreadPriority(true, 10);

		robot.getAutonomousChooser().getChooser().onChange((autonomousCommand) -> {
			this.autonomousCommand = autonomousCommand.get();
		});
	}

	@Override
	public void disabledInit() {
		if (!DriverStationUtil.isMatch()) {
			BrakeStateManager.coast();
		}
	}

	@Override
	public void disabledExit() {
		if (!DriverStationUtil.isMatch()) {
			BrakeStateManager.brake();
		}
	}

	@Override
	public void autonomousInit() {
		autonomousStartTimeSeconds = TimeUtil.getCurrentTimeSeconds();
		ballCounter = 0;
		lastBallShotTimestamp = -1;
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

	public static double getAutonomousStartTimeSeconds(){
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
		updateBallCounter();
		robot.periodic();
		AlertManager.reportAlerts();
	}

	private void updateTimeRelatedData() {
		roborioCycles++;
		Logger.recordOutput("RoborioCycles", roborioCycles);
		TimeUtil.updateCycleTime(roborioCycles);
	}

	private void updateBallCounter() {
		if (lastBallShotTimestamp == -1) {
			if (robot.getRobotCommander().getShooterStateHandler().hasABallBeenShot()) {
				if (DriverStation.isAutonomous()) {
					lastBallShotTimestamp = TimeUtil.getTimeSinceAutonomousInitSeconds();
                } else {
					lastBallShotTimestamp = TimeUtil.getTimeSinceTeleopInitSeconds();
                }
                ballCounter++;
            }
		}
		else {
			if (DriverStation.isAutonomous()) {
				if (TimeUtil.getTimeSinceAutonomousInitSeconds() - lastBallShotTimestamp >= FlywheelConstants.FLYWHEEL_SHOOT_DROP_IN_VELOCITY_ROTATIONS) {
					lastBallShotTimestamp = TimeUtil.getTimeSinceAutonomousInitSeconds();
					ballCounter++;
				}
			} else {
				if (TimeUtil.getTimeSinceTeleopInitSeconds() - lastBallShotTimestamp >= FlywheelConstants.TIME_BETWEEN_CHECKS_TO_COUNT_BALLS) {
					lastBallShotTimestamp = TimeUtil.getTimeSinceTeleopInitSeconds();
					ballCounter++;
				}
			}
		}
		Logger.recordOutput("BallCounter", ballCounter);
	}

}
