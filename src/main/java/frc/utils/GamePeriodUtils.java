package frc.utils;

import frc.utils.driverstation.DriverStationUtil;
import frc.utils.time.TimeUtil;

public class GamePeriodUtils {

	public static final int TRANSITION_SHIFT_DURATION_SECONDS = 10;
	public static final int ALLIANCE_SHIFT_DURATION_SECONDS = 25;
	public static final int ENDGAME_START_TIME_SECONDS = 110;
	public static final int ENDGAME_DURATION_SECONDS = 30;
	public static final int GAME_DURATION_SECONDS = ENDGAME_START_TIME_SECONDS + ENDGAME_DURATION_SECONDS;
	public static final int SECONDS_TO_SCORE_AFTER_GAME_ENDS = 3;
	public static final int ACTIVE_HUB_TIME_AFTER_GAME_ENDS_SECONDS = GAME_DURATION_SECONDS + SECONDS_TO_SCORE_AFTER_GAME_ENDS;
	public static final double ROBOT_TIMER_DELAY = 1.5;

	public static boolean isTransitionShift() {
		if (!DriverStationUtil.isTeleop()) {
			return false;
		}
		return TimeUtil.getTimeSinceTeleopInitSeconds() <= TRANSITION_SHIFT_DURATION_SECONDS;
	}

	public static boolean hasGameEnded() {
		return TimeUtil.getTimeSinceTeleopInitSeconds() >= GAME_DURATION_SECONDS;
	}

	public static boolean isInEndgame() {
		return TimeUtil.getTimeSinceTeleopInitSeconds() >= ENDGAME_START_TIME_SECONDS
			&& TimeUtil.getTimeSinceTeleopInitSeconds() <= ACTIVE_HUB_TIME_AFTER_GAME_ENDS_SECONDS;
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
		return GAME_DURATION_SECONDS - TimeUtil.getTimeSinceTeleopInitSeconds();
	}

	public static double getTimeUntilShiftEnds() {
		if (!DriverStationUtil.isTeleop()) {
			return -1;
		}
		return ALLIANCE_SHIFT_DURATION_SECONDS
			- ((TimeUtil.getTimeSinceTeleopInitSeconds() - TRANSITION_SHIFT_DURATION_SECONDS) % GamePeriodUtils.ALLIANCE_SHIFT_DURATION_SECONDS);
	}

}
