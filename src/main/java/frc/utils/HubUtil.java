package frc.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.driverstation.GameSpecificMessageResponse;
import frc.RobotManager;
import frc.constants.GamePeriodConstants;
import frc.utils.driverstation.DriverStationUtil;
import frc.utils.time.TimeUtil;

import java.util.Optional;


public class HubUtil {

	public static Optional<DriverStation.Alliance> getAutoWinnerAlliance() {
		String gameData = DriverStation.getGameSpecificMessage();
		if (gameData.isEmpty()) {
			new Alert("Unknown starting alliance", Alert.AlertType.kWarning).set(true);
			return Optional.empty();
		}
		Optional<DriverStation.Alliance> alliance = switch (GameSpecificMessageResponse.responseToEnum(gameData.charAt(0))) {
			case BLUE -> Optional.of(DriverStation.Alliance.Blue);
			case RED -> Optional.of(DriverStation.Alliance.Red);
			default -> Optional.empty();
		};
		if (alliance.isEmpty()) {
			new Alert("Unknown starting alliance", Alert.AlertType.kWarning).set(true);
			return Optional.empty();
		}
		return alliance;
	}

	public static double getTimeSinceTeleopInitSeconds() {
		return TimeUtil.getCurrentTimeSeconds() - RobotManager.getTeleopStartTimeSeconds();
	}

	public static Optional<DriverStation.Alliance> getStartingAlliance() {
		if (getAutoWinnerAlliance().isPresent()) {
			return switch (getAutoWinnerAlliance().get()) {
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
		return (int) (getTimeSinceTeleopInitSeconds() - GamePeriodConstants.TRANSITION_SHIFT_TIME_SECONDS)
			/ GamePeriodConstants.ALLIANCE_SHIFT_LENGTH_SECONDS;
	}

	public static Optional<DriverStation.Alliance> getActiveHub() {
		if (DriverStation.isAutonomous()) {
			return DriverStationUtil.getAlliance();
		} else if (!DriverStation.isTeleop()) {
			return Optional.empty();
		} else if (DriverStationUtil.isTransitionShift()) {
			return DriverStationUtil.getAlliance();
		}

		return isAutoWinnerShift() ? getAutoWinnerAlliance() : getStartingAlliance();
	}

	public static boolean isMyHubActive() {
		return getActiveHub().equals(DriverStationUtil.getAlliance());
	}

	public static double timeUntilCurrentShiftEndsSeconds() {
		if (DriverStationUtil.isTransitionShift()) {
			return DriverStationUtil.timeUntilTransitionShiftEnds();
		} else if (DriverStationUtil.hasEndGameStarted()) {
			return DriverStationUtil.timeUntilEndgameEnds();
		}
		return DriverStationUtil.timeUntilShiftEnds();
	}

	public static double getTimeLeftUntilActive() {
		if (isMyHubActive()) {
			return 0.0;
		}
		return timeUntilCurrentShiftEndsSeconds();
	}

	public static double getTimeLeftUntilInactive() {
		if (!isMyHubActive() || DriverStationUtil.hasGameEnded()) {
			return 0.0;
		}
		return timeUntilCurrentShiftEndsSeconds();
	}

	public static boolean isRobotAllianceAutoWinner() {
		return DriverStationUtil.getAlliance() == getAutoWinnerAlliance();
	}

}
