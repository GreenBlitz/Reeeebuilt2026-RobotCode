package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.driverstation.DriverStationUtil;
import frc.utils.time.TimeUtil;

public class GamePeriodUtils {

	public static final int TRANSITION_SHIFT_DURATION_SECONDS = 10;
	public static final int ALLIANCE_SHIFT_DURATION_SECONDS = 25;
	public static final int ENDGAME_DURATION_SECONDS_SINCE_TELEOP = 110;
	public static final int ENDGAME_DURATION_SECONDS = 30;
	public static final int GAME_END_DURATION_SECONDS = ENDGAME_DURATION_SECONDS_SINCE_TELEOP + ENDGAME_DURATION_SECONDS;

	public static boolean isTransitionShift() {
		if (!DriverStation.isTeleop()) {
			return false;
		}
		return TimeUtil.getTimeSinceTeleopInitSeconds() <= TRANSITION_SHIFT_DURATION_SECONDS;
	}

	public static boolean hasGameEnded() {
		return TimeUtil.getTimeSinceTeleopInitSeconds() >= GAME_END_DURATION_SECONDS;
	}

	public static boolean hasEndGameStarted() {
		return TimeUtil.getTimeSinceTeleopInitSeconds() >= ENDGAME_DURATION_SECONDS_SINCE_TELEOP;
	}

	public static double getTimeUntilTransitionShiftEnds() {
		if (!isTransitionShift()) {
			return -1;
		}
		return TRANSITION_SHIFT_DURATION_SECONDS - TimeUtil.getTimeSinceTeleopInitSeconds();
	}

	public static double getTimeUntilGameEnds() {
		if (!DriverStationUtil.isTeleop()) {
			return -1;
		}
		return GAME_END_DURATION_SECONDS - TimeUtil.getTimeSinceTeleopInitSeconds();
	}

	public static double getTimeUntilShiftEnds() {
		if (!DriverStation.isTeleop()) {
			return -1;
		}
		return ALLIANCE_SHIFT_DURATION_SECONDS
			- ((TimeUtil.getTimeSinceTeleopInitSeconds() - TRANSITION_SHIFT_DURATION_SECONDS) % GamePeriodUtils.ALLIANCE_SHIFT_DURATION_SECONDS);
	}

}
