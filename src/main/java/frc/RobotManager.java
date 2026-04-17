// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.revrobotics.util.StatusLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.constants.field.Field;
import frc.robot.Robot;
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
	private PathPlannerAutoWrapper autonomousCommand;
	private int roborioCycles;
	private static double teleopStartTimeSeconds = -1;
	private final Field2d field2d;

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

		field2d = new Field2d();
		SmartDashboard.putData(field2d);

		robot.getAutonomousChooser().getChooser().onChange((autonomousCommand) -> {
			this.autonomousCommand = autonomousCommand.get();
			field2d.getObject("path").setPoses(robot.getAutonomousChooser().getChosenValue().getPath(!Field.isFieldConventionAlliance()));
		});
	}

	@Override
	public void disabledExit() {
		if (!DriverStationUtil.isMatch()) {
			BrakeStateManager.setBrakeMode(BrakeMode.BRAKE);
		}
	}

	@Override
	public void autonomousInit() {
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
		Logger.recordOutput("averagePeriodPBS/Autonomous", robot.getAverageBPSForLastXSeconds(GamePeriodUtils.AUTONOMOUS_DURATION_SECONDS));
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
		logElasticRelatedInfo();
	}

	private void updateTimeRelatedData() {
		roborioCycles++;
		Logger.recordOutput("RoborioCycles", roborioCycles);
		TimeUtil.updateCycleTime(roborioCycles);
	}

	private void logElasticRelatedInfo() {
		Logger.recordOutput("TimeUntilHubActive", HubUtil.getTimeLeftUntilActiveSeconds(TimeUtil.getTimeSinceTeleopInitSeconds()));
		Logger.recordOutput("TimeUntilHubInactive", HubUtil.getTimeLeftUntilInactiveSeconds(TimeUtil.getTimeSinceTeleopInitSeconds()));
		Logger.recordOutput("IsHubActive", HubUtil.isOurHubActive(TimeUtil.getTimeSinceTeleopInitSeconds()));
		Logger.recordOutput("TimeSinceTeleop", TimeUtil.getTimeSinceTeleopInitSeconds());
		Logger.recordOutput("CurrentGamePeriod", GamePeriodUtils.getCurrentGamePeriod());

		field2d.setRobotPose(robot.getPoseEstimator().getEstimatedPose());
	}

}
