package frc.utils;

import frc.utils.driverstation.DriverStationUtil;
import frc.utils.time.TimeUtil;

public class GamePeriodUtils {

	public static final int TRANSITION_SHIFT_DURATION_SECONDS = 10;
	public static final int ALLIANCE_SHIFT_DURATION_SECONDS = 25;
	public static final int TELEOP_DURATION_SECONDS = 110;
	public static final int ENDGAME_DURATION_SECONDS = 30;
	public static final int GAME_DURATION_SECONDS = TELEOP_DURATION_SECONDS + ENDGAME_DURATION_SECONDS;

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
		return TimeUtil.getTimeSinceTeleopInitSeconds() >= TELEOP_DURATION_SECONDS
			&& TimeUtil.getTimeSinceTeleopInitSeconds() <= GAME_DURATION_SECONDS;
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
