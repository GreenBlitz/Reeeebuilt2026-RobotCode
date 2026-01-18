package frc.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.RobotManager;
import frc.constants.GameConstants;
import frc.utils.time.TimeUtil;

public class HubUtil {

	public static DriverStation.Alliance getAutoWinner() {
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

	public static boolean isCurrentShiftOfStartingAlliance(int isCurrentShiftOfStartingAlliance) {
		return isCurrentShiftOfStartingAlliance != 0;
	}

	public static DriverStation.Alliance oppositeAllianceOfStartingAlliance(DriverStation.Alliance startingAlliance) {
		switch (startingAlliance) {
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
		}
		if (RobotManager.getTeleopStartTime() == 0) {
			new Alert("Unknown time since teleop started", Alert.AlertType.kInfo);
			return DriverStationUtil.DEFAULT_ALLIANCE;
		}
		if (DriverStation.isTeleop()) {
			double howMuchTimeSinceTeleopInit = (TimeUtil.getCurrentTimeSeconds() - (RobotManager.getTeleopStartTime()));

			if (howMuchTimeSinceTeleopInit <= GameConstants.TRANSITION_SHIFT_TIME) {
				return DriverStationUtil.getAlliance();
			}

			double howMuchShiftsPassed = (howMuchTimeSinceTeleopInit - GameConstants.TRANSITION_SHIFT_TIME) / GameConstants.SHIFT_LENGTH;
			int isTheCurrentShiftAShiftOfTheStartingAlliance = (int) Math.floor(howMuchShiftsPassed)
				% GameConstants.FIND_IF_IT_IS_A_SHIFT_OF_THE_STARTING_ALLIANCE_BY_MODULO;

			if (howMuchTimeSinceTeleopInit >= GameConstants.ENDGAME_END) {
				return DriverStationUtil.DEFAULT_ALLIANCE;
			} else if (howMuchTimeSinceTeleopInit >= GameConstants.ENDGAME_START) {
				return DriverStationUtil.getAlliance();
			}

			DriverStation.Alliance autoWinningAlliance = /* getStartingAlliance() */ DriverStation.Alliance.Blue;
			return isCurrentShiftOfStartingAlliance(isTheCurrentShiftAShiftOfTheStartingAlliance)
				? autoWinningAlliance
				: oppositeAllianceOfStartingAlliance(autoWinningAlliance);
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
			return GameConstants.SHIFT_LENGTH
				- ((howMuchTimePassedSinceTeleopInit - GameConstants.TRANSITION_SHIFT_TIME) % GameConstants.SHIFT_LENGTH);
		}
	}

	public static double howMuchTimeLeftForMyHub() {
		double howMuchTimePassedSinceTeleopInit = TimeUtil.getCurrentTimeSeconds() - RobotManager.getTeleopStartTime();
		if (!isMyHubActive()) {
			return 0;
		} else if (howMuchTimePassedSinceTeleopInit <= GameConstants.TRANSITION_SHIFT_TIME) {
			return GameConstants.TRANSITION_SHIFT_TIME - (howMuchTimePassedSinceTeleopInit);
		} else if (howMuchTimePassedSinceTeleopInit >= GameConstants.ENDGAME_END) {
			return 0;
		} else if (howMuchTimePassedSinceTeleopInit >= GameConstants.ENDGAME_START) {
			return GameConstants.ENDGAME_LENGTH - (howMuchTimePassedSinceTeleopInit - GameConstants.ENDGAME_START);
		} else {
			return GameConstants.SHIFT_LENGTH
				- ((howMuchTimePassedSinceTeleopInit - GameConstants.TRANSITION_SHIFT_TIME) % GameConstants.SHIFT_LENGTH);
		}
	}

}
