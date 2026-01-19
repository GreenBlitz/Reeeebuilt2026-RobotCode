package frc.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.RobotManager;
import frc.constants.GameConstants;
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
			new Alert("Unknown starting alliance", Alert.AlertType.kWarning);
			return DriverStationUtil.DEFAULT_ALLIANCE;
		}
		return alliance;
	}

	public static boolean isStartingAlliancesShift(int startingAllianceShift) {
		return startingAllianceShift != 0;
	}

	public static DriverStation.Alliance getStartingAlliance() {
		return switch (getAutoWinnerAlliance()) {
			case Red -> DriverStation.Alliance.Blue;
			case Blue -> DriverStation.Alliance.Red;
			default -> DriverStationUtil.DEFAULT_ALLIANCE;
		};
	}

	public static DriverStation.Alliance getActiveHub() {
		if (DriverStation.isAutonomous()) {
			return DriverStationUtil.getAlliance();
		} else if (DriverStation.isTeleop()) {
			double howMuchTimeSinceTeleopInitSeconds = (TimeUtil.getCurrentTimeSeconds() - (RobotManager.getTeleopStartTime()));

			if (howMuchTimeSinceTeleopInitSeconds <= GameConstants.TRANSITION_SHIFT_TIME_SECONDS) {
				return DriverStationUtil.getAlliance();
			}

			double howManyShiftsPassed = (howMuchTimeSinceTeleopInitSeconds - GameConstants.TRANSITION_SHIFT_TIME_SECONDS)
				/ GameConstants.ALLIANCE_SHIFT_LENGTH_SECONDS;
			boolean isShiftOfStartingAlliance = ((int) Math.floor(howManyShiftsPassed) % 2) != 0;

			if (howMuchTimeSinceTeleopInitSeconds >= GameConstants.ENDGAME_END_TIME_SECONDS) {
				return DriverStationUtil.DEFAULT_ALLIANCE;
			} else if (howMuchTimeSinceTeleopInitSeconds >= GameConstants.ENDGAME_START_TIME_SECONDS_SINCE_TELEOP) {
				return DriverStationUtil.getAlliance();
			}

			DriverStation.Alliance autoWinningAlliance = getAutoWinnerAlliance();
			return isShiftOfStartingAlliance ? autoWinningAlliance : getStartingAlliance();
		}
		return DriverStationUtil.DEFAULT_ALLIANCE;
	}

	public static boolean isMyHubActive() {
		return getActiveHub() == DriverStationUtil.getAlliance();
	}

	public static double timeUntilCurrentShiftEnds() {
		double howMuchTImePassedSinceTeleopInit = TimeUtil.getCurrentTimeSeconds() - RobotManager.getTeleopStartTime();
		if (howMuchTImePassedSinceTeleopInit <= GameConstants.TRANSITION_SHIFT_TIME_SECONDS) {
			return GameConstants.TRANSITION_SHIFT_TIME_SECONDS - howMuchTImePassedSinceTeleopInit;
		} else if (howMuchTImePassedSinceTeleopInit >= GameConstants.ENDGAME_START_TIME_SECONDS_SINCE_TELEOP) {
			return GameConstants.ENDGAME_LENGTH_SECONDS
				- (howMuchTImePassedSinceTeleopInit - GameConstants.ENDGAME_START_TIME_SECONDS_SINCE_TELEOP);
		}
		return GameConstants.ALLIANCE_SHIFT_LENGTH_SECONDS
			- ((howMuchTImePassedSinceTeleopInit - GameConstants.TRANSITION_SHIFT_TIME_SECONDS) % GameConstants.ALLIANCE_SHIFT_LENGTH_SECONDS);
	}

	public static double countUntilAllianceHubIsActiveSeconds() {
		if (isMyHubActive()) {
			return 0;
		} else {
			return timeUntilCurrentShiftEnds();
		}
	}

	public static double howMuchTimeLeftForMyActiveHub() {
		double howMuchTimePassedSinceTeleopInit = TimeUtil.getCurrentTimeSeconds() - RobotManager.getTeleopStartTime();
		if (!isMyHubActive() || howMuchTimePassedSinceTeleopInit >= GameConstants.ENDGAME_END_TIME_SECONDS) {
			return 0;
		}
		return timeUntilCurrentShiftEnds();
	}

}
