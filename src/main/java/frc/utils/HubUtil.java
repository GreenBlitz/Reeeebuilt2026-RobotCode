package frc.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.RobotManager;
import frc.constants.GameConstants;
import frc.utils.time.TimeUtil;

public class HubUtil {

	public static DriverStation.Alliance getAutoWinnerAlliance() {
		String gameData = DriverStation.getGameSpecificMessage();
		if (gameData.isEmpty()) {
			new Alert("Unknown starting alliance", Alert.AlertType.kWarning);
			return DriverStationUtil.DEFAULT_ALLIANCE;
		}
		DriverStation.Alliance alliance = switch (gameData.charAt(0)) {
			case 'B' -> DriverStation.Alliance.Blue;
			case 'R' -> DriverStation.Alliance.Red;
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

	public static DriverStation.Alliance oppositeAllianceOfStartingAlliance() {
		switch (getAutoWinnerAlliance()) {
			case Red -> {
				return DriverStation.Alliance.Blue;
			}
			case Blue -> {
				return DriverStation.Alliance.Red;
			}
		}
		return DriverStationUtil.DEFAULT_ALLIANCE;
	}

	public static DriverStation.Alliance whichHubIsActive() {
		if (DriverStation.isAutonomous()) {
			return DriverStationUtil.getAlliance();
		} else if (DriverStation.isTeleop()) {
			double howMuchTimeSinceTeleopInitSeconds = (TimeUtil.getCurrentTimeSeconds() - (RobotManager.getTeleopStartTime()));

			if (howMuchTimeSinceTeleopInitSeconds <= GameConstants.TRANSITION_SHIFT_TIME_SECONDS) {
				return DriverStationUtil.getAlliance();
			}

			double howMuchShiftsPassed = (howMuchTimeSinceTeleopInitSeconds - GameConstants.TRANSITION_SHIFT_TIME_SECONDS)
				/ GameConstants.ALLIANCE_SHIFT_LENGTH_SECONDS;
			boolean isShiftOfStartingAlliance = (int) Math.floor(howMuchShiftsPassed) % GameConstants.STARTING_ALLIANCE_SHIFT_CYCLE != 0;

			if (howMuchTimeSinceTeleopInitSeconds >= GameConstants.ENDGAME_END_TIME_SECONDS) {
				return DriverStationUtil.DEFAULT_ALLIANCE;
			} else if (howMuchTimeSinceTeleopInitSeconds >= GameConstants.ENDGAME_START_TIME_SECONDS) {
				return DriverStationUtil.getAlliance();
			}

			DriverStation.Alliance autoWinningAlliance = getAutoWinnerAlliance();
			return isShiftOfStartingAlliance ? autoWinningAlliance : oppositeAllianceOfStartingAlliance();
		}
		return DriverStationUtil.DEFAULT_ALLIANCE;
	}

	public static boolean isMyHubActive() {
		return whichHubIsActive() == DriverStationUtil.getAlliance();
	}

	public static double whenWillMyHubBeActive() {
		double howMuchTimePassedSinceTeleopInit = TimeUtil.getCurrentTimeSeconds() - RobotManager.getTeleopStartTime();
		if (isMyHubActive()) {
			return 0;
		} else {
			double timeUntilActive = GameConstants.ALLIANCE_SHIFT_LENGTH_SECONDS
				- ((howMuchTimePassedSinceTeleopInit - GameConstants.TRANSITION_SHIFT_TIME_SECONDS)
					% GameConstants.ALLIANCE_SHIFT_LENGTH_SECONDS);
			return timeUntilActive;
		}
	}

	public static double howMuchTimeLeftForMyHub() {
		double howMuchTimePassedSinceTeleopInit = TimeUtil.getCurrentTimeSeconds() - RobotManager.getTeleopStartTime();
		if (!isMyHubActive() || howMuchTimePassedSinceTeleopInit >= GameConstants.ENDGAME_END_TIME_SECONDS) {
			return 0;
		} else if (howMuchTimePassedSinceTeleopInit <= GameConstants.TRANSITION_SHIFT_TIME_SECONDS) {
			return GameConstants.TRANSITION_SHIFT_TIME_SECONDS - (howMuchTimePassedSinceTeleopInit);
		} else if (howMuchTimePassedSinceTeleopInit >= GameConstants.ENDGAME_START_TIME_SECONDS) {
			return GameConstants.ENDGAME_LENGTH_SECONDS - (howMuchTimePassedSinceTeleopInit - GameConstants.ENDGAME_START_TIME_SECONDS);
		}
		double timeLeftUntilInactive = GameConstants.ALLIANCE_SHIFT_LENGTH_SECONDS
			- ((howMuchTimePassedSinceTeleopInit - GameConstants.TRANSITION_SHIFT_TIME_SECONDS) % GameConstants.ALLIANCE_SHIFT_LENGTH_SECONDS);
		return timeLeftUntilInactive;
	}

}
