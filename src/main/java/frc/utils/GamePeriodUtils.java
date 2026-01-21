package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.time.TimeUtil;

public class GamePeriodUtils {

	public static final int TRANSITION_SHIFT_TIME_SECONDS = 10;
	public static final int ALLIANCE_SHIFT_LENGTH_SECONDS = 25;
	public static final int ENDGAME_START_TIME_SECONDS_SINCE_TELEOP = 110;
	public static final int ENDGAME_LENGTH_SECONDS = 30;
	public static final int GAME_END_TIME_SECONDS = ENDGAME_START_TIME_SECONDS_SINCE_TELEOP + ENDGAME_LENGTH_SECONDS;

	public static boolean isTransitionShift() {
		if (TimeUtil.getTimeSinceTeleopInitSeconds() == -1) {
			return false;
		}
		return TimeUtil.getTimeSinceTeleopInitSeconds() <= TRANSITION_SHIFT_TIME_SECONDS;
	}

	public static boolean hasGameEnded() {
		return TimeUtil.getTimeSinceTeleopInitSeconds() >= GAME_END_TIME_SECONDS;
	}

	public static boolean hasEndGameStarted() {
		if (TimeUtil.getTimeSinceTeleopInitSeconds() == -1) {
			return false;
		}
		return TimeUtil.getTimeSinceTeleopInitSeconds() >= ENDGAME_START_TIME_SECONDS_SINCE_TELEOP;
	}

	public static double getTimeUntilTransitionShiftEnds() {
		if (isTransitionShift()) {
			return TRANSITION_SHIFT_TIME_SECONDS - TimeUtil.getTimeSinceTeleopInitSeconds();
		}
		return -1;
	}

	public static double getTimeUntilEndgameEnds() {
		if (hasEndGameStarted()) {
			return ENDGAME_LENGTH_SECONDS - (TimeUtil.getTimeSinceTeleopInitSeconds() - ENDGAME_START_TIME_SECONDS_SINCE_TELEOP);
		}
		return -1;
	}

	public static double getTimeUntilShiftEnds() {
		if (DriverStation.isTeleop()) {
			return ALLIANCE_SHIFT_LENGTH_SECONDS
				- ((TimeUtil.getTimeSinceTeleopInitSeconds() - TRANSITION_SHIFT_TIME_SECONDS) % GamePeriodUtils.ALLIANCE_SHIFT_LENGTH_SECONDS);
		}
		return -1;
	}

}
