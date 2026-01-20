package frc.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.RobotManager;
import frc.constants.GamePeriodConstants;
import frc.utils.time.TimeUtil;

public class HubUtil {

	private static final char RED = 'R';
	private static final char BLUE = 'B';

	public static DriverStation.Alliance getAutoWinnerAlliance() {
		String gameData = DriverStation.getGameSpecificMessage();
		if (gameData.isEmpty()) {
			new Alert("Unknown starting alliance", Alert.AlertType.kWarning);
			return DriverStationUtil.DEFAULT_ALLIANCE;
		}
		DriverStation.Alliance alliance = switch (gameData.charAt(0)) {
			case BLUE -> DriverStation.Alliance.Blue;
			case RED -> DriverStation.Alliance.Red;
			default -> null;
		};
		if (alliance == null) {
			new Alert("Unknown starting alliance", Alert.AlertType.kWarning).set(true);
			return DriverStationUtil.DEFAULT_ALLIANCE;
		}
		return alliance;
	}

	public static double getTimeSinceTeleopInitSeconds() {
		return TimeUtil.getCurrentTimeSeconds() - RobotManager.getTeleopStartTimeSeconds();
	}

	public static DriverStation.Alliance isShiftOfStartingAlliance() {
		return switch (getAutoWinnerAlliance()) {
			case Red -> DriverStation.Alliance.Blue;
			case Blue -> DriverStation.Alliance.Red;
			default -> DriverStationUtil.DEFAULT_ALLIANCE;
		};
	}

	public static DriverStation.Alliance getActiveHub() {
		if (DriverStation.isAutonomous()) {
			return DriverStationUtil.getAlliance();
		} else if (!DriverStation.isTeleop()) {
			return DriverStationUtil.DEFAULT_ALLIANCE;
		}
		double timeSinceTeleopInitSeconds = getTimeSinceTeleopInitSeconds();

		if (timeSinceTeleopInitSeconds <= GamePeriodConstants.TRANSITION_SHIFT_TIME_SECONDS) {
			return DriverStationUtil.getAlliance();
		}

		int shiftsPassed = (int) (timeSinceTeleopInitSeconds - GamePeriodConstants.TRANSITION_SHIFT_TIME_SECONDS)
			/ GamePeriodConstants.ALLIANCE_SHIFT_LENGTH_SECONDS;
		boolean isAutoWinnerShift = (shiftsPassed % 2) != 0;

		if (timeSinceTeleopInitSeconds >= GamePeriodConstants.GAME_END_TIME_SECONDS) {
			return DriverStationUtil.DEFAULT_ALLIANCE;
		} else if (timeSinceTeleopInitSeconds >= GamePeriodConstants.ENDGAME_START_TIME_SECONDS_SINCE_TELEOP) {
			return DriverStationUtil.getAlliance();
		}

		return isAutoWinnerShift ? getAutoWinnerAlliance() : isShiftOfStartingAlliance();
	}

	public static boolean isMyHubActive() {
		return getActiveHub() == DriverStationUtil.getAlliance();
	}

	public static double timeUntilCurrentShiftEndsSeconds() {
		double timePassedSinceTeleopInit = getTimeSinceTeleopInitSeconds();
		if (timePassedSinceTeleopInit <= GamePeriodConstants.TRANSITION_SHIFT_TIME_SECONDS) {
			return GamePeriodConstants.TRANSITION_SHIFT_TIME_SECONDS - timePassedSinceTeleopInit;
		} else if (timePassedSinceTeleopInit >= GamePeriodConstants.ENDGAME_START_TIME_SECONDS_SINCE_TELEOP) {
			return GamePeriodConstants.ENDGAME_LENGTH_SECONDS
				- (timePassedSinceTeleopInit - GamePeriodConstants.ENDGAME_START_TIME_SECONDS_SINCE_TELEOP);
		}
		return GamePeriodConstants.ALLIANCE_SHIFT_LENGTH_SECONDS
			- ((timePassedSinceTeleopInit - GamePeriodConstants.TRANSITION_SHIFT_TIME_SECONDS)
				% GamePeriodConstants.ALLIANCE_SHIFT_LENGTH_SECONDS);
	}

	public static double getTimeLeftUntilActive() {
		if (isMyHubActive()) {
			return 0;
		}
		return timeUntilCurrentShiftEndsSeconds();
	}

	public static double getTimeLeftUntilInactive() {
		double timePassedSinceTeleopInit = getTimeSinceTeleopInitSeconds();
		if (!isMyHubActive() || timePassedSinceTeleopInit >= GamePeriodConstants.GAME_END_TIME_SECONDS) {
			return 0;
		}
		return timeUntilCurrentShiftEndsSeconds();
	}

	public static boolean isRobotAllianceAutoWinner() {
		return DriverStationUtil.getAlliance() == getAutoWinnerAlliance();
	}

}
