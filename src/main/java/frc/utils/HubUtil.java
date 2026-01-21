package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.driverstation.GameSpecificMessageResponse;
import frc.RobotManager;
import frc.utils.driverstation.DriverStationUtil;
import frc.utils.time.TimeUtil;
import frc.utils.alerts.Alert;

import java.util.Optional;


public class HubUtil {

	public static Optional<DriverStation.Alliance> autoWinnerAlliance = getAutoWinnerAlliance();
	public static Optional<DriverStation.Alliance> autoLosingAlliance = getAutoLosingAlliance();

	public static Optional<DriverStation.Alliance> getAutoWinnerAlliance() {
		String gameData = DriverStation.getGameSpecificMessage();
		if (gameData.isEmpty()) {
			alertWarningForEmptyAlliance("Unknown auto winner alliance");
		}
		Optional<DriverStation.Alliance> alliance = switch (GameSpecificMessageResponse.responseToEnum(gameData.charAt(0))) {
			case BLUE -> Optional.of(DriverStation.Alliance.Blue);
			case RED -> Optional.of(DriverStation.Alliance.Red);
			case EMPTY -> Optional.empty();
		};
		if (alliance.isEmpty()) {
			alertWarningForEmptyAlliance("Didnt get auto winning alliance");
		}
		return alliance;
	}

	public static Optional<DriverStation.Alliance> alertWarningForEmptyAlliance(String name) {
		new Alert(Alert.AlertType.WARNING, name).report();
		return Optional.empty();
	}

	public static void refreshAlliances() {
		if (autoWinnerAlliance.isEmpty()) {
			autoWinnerAlliance = getAutoWinnerAlliance();
			autoLosingAlliance = getAutoLosingAlliance();
		}
	}

	public static double getTimeSinceTeleopInitSeconds() {
		return TimeUtil.getCurrentTimeSeconds() - RobotManager.getTeleopStartTimeSeconds();
	}

	public static Optional<DriverStation.Alliance> getAutoLosingAlliance() {
		if (autoWinnerAlliance.isPresent()) {
			return switch (autoWinnerAlliance.get()) {
				case Red -> Optional.of(DriverStation.Alliance.Blue);
				case Blue -> Optional.of(DriverStation.Alliance.Red);
			};
		}
		return Optional.empty();
	}

	public static boolean isAutoWinnerShift() {
		return getShiftsPassed() % 2 != 0;
	}

	public static int getShiftsPassed() {
		return (int) (getTimeSinceTeleopInitSeconds() - GamePeriodUtils.TRANSITION_SHIFT_TIME_SECONDS)
			/ GamePeriodUtils.ALLIANCE_SHIFT_LENGTH_SECONDS;
	}

	public static Optional<DriverStation.Alliance> getActiveHub() {
		if (DriverStation.isAutonomous()) {
			return DriverStationUtil.getAlliance();
		} else if (!DriverStation.isTeleop() || GamePeriodUtils.hasGameEnded()) {
			return Optional.empty();
		} else if (GamePeriodUtils.isTransitionShift() || GamePeriodUtils.hasEndGameStarted()) {
			return DriverStationUtil.getAlliance();
		}

		return isAutoWinnerShift() ? autoWinnerAlliance : autoLosingAlliance;
	}

	public static boolean isOurHubActive() {
		return getActiveHub().equals(DriverStationUtil.getAlliance());
	}

	public static double timeUntilCurrentShiftEndsSeconds() {
		if (GamePeriodUtils.isTransitionShift()) {
			return GamePeriodUtils.timeUntilTransitionShiftEnds();
		} else if (GamePeriodUtils.hasEndGameStarted()) {
			return GamePeriodUtils.timeUntilEndgameEnds();
		}
		return GamePeriodUtils.timeUntilShiftEnds();
	}

	public static double getTimeLeftUntilActive() {
		if (isOurHubActive()) {
			return 0.0;
		}
		return timeUntilCurrentShiftEndsSeconds();
	}

	public static double getTimeLeftUntilInactive() {
		if (!isOurHubActive() || GamePeriodUtils.hasGameEnded()) {
			return 0.0;
		}
		return timeUntilCurrentShiftEndsSeconds();
	}

	public static boolean isRobotAllianceAutoWinner() {
		return DriverStationUtil.getAlliance() == autoWinnerAlliance;
	}

}
