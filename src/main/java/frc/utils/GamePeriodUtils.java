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
		return TimeUtil.getTimeSinceTeleopInitSeconds() <= GamePeriodUtils.TRANSITION_SHIFT_TIME_SECONDS;
	}

	public static boolean hasGameEnded() {
		return TimeUtil.getTimeSinceTeleopInitSeconds() >= GamePeriodUtils.GAME_END_TIME_SECONDS;
	}

	public static boolean hasEndGameStarted() {
		return TimeUtil.getTimeSinceTeleopInitSeconds() >= GamePeriodUtils.ENDGAME_START_TIME_SECONDS_SINCE_TELEOP;
	}

	public static double timeUntilTransitionShiftEnds() {
		if (GamePeriodUtils.isTransitionShift()) {
			return GamePeriodUtils.TRANSITION_SHIFT_TIME_SECONDS - TimeUtil.getTimeSinceTeleopInitSeconds();
		}
		return -1;
	}

	public static double timeUntilEndgameEnds() {
		if (GamePeriodUtils.hasEndGameStarted()) {
			return GamePeriodUtils.ENDGAME_LENGTH_SECONDS
				- (TimeUtil.getTimeSinceTeleopInitSeconds() - GamePeriodUtils.ENDGAME_START_TIME_SECONDS_SINCE_TELEOP);
		}
		return -1;
	}

	public static double timeUntilShiftEnds() {
		if (DriverStation.isTeleop()) {
			return GamePeriodUtils.ALLIANCE_SHIFT_LENGTH_SECONDS
				- ((TimeUtil.getTimeSinceTeleopInitSeconds() - GamePeriodUtils.TRANSITION_SHIFT_TIME_SECONDS)
					% GamePeriodUtils.ALLIANCE_SHIFT_LENGTH_SECONDS);
		}
		return -1;
	}


}
