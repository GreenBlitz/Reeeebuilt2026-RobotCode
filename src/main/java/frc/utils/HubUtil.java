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

	public static Optional<DriverStation.Alliance> isShiftOfStartingAlliance() {
		if (getAutoWinnerAlliance().isPresent()) {
			return switch (getAutoWinnerAlliance().get()) {
				case Red -> Optional.of(DriverStation.Alliance.Blue);
				case Blue -> Optional.of(DriverStation.Alliance.Red);
				default -> Optional.empty();
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
		if (DriverStationUtil.isAuto().isPresent()) {
			return DriverStationUtil.isAuto();
		} else if (DriverStationUtil.isNotTeleop().isPresent()) {
			return Optional.empty();
		} else if (DriverStationUtil.isDisabled()) {
			return Optional.empty();
		}

		if (DriverStationUtil.isTransitionShift().isPresent()) {
			return DriverStationUtil.isTransitionShift();
		} else if (DriverStationUtil.hasGameEnded().isPresent()) {
			return Optional.empty();
		} else if (DriverStationUtil.hasEndGameStarted().isPresent()) {
			return DriverStationUtil.hasEndGameStarted();
		}

		if (getAutoWinnerAlliance().isPresent() && isShiftOfStartingAlliance().isPresent()) {
//			return isAutoWinnerShift() ? getAutoWinnerAlliance() : isShiftOfStartingAlliance();
			return Optional.of(DriverStation.Alliance.Blue);
		}
		return Optional.empty();
	}

	public static Optional<Boolean> isMyHubActive() {
		if (getActiveHub().isPresent()) {
			return Optional.of(getActiveHub() == DriverStationUtil.getAlliance());
		}
		return Optional.empty();
	}

	public static double timeUntilCurrentShiftEndsSeconds() {
		if (DriverStationUtil.isTransitionShift().isPresent()) {
			return DriverStationUtil.timeUntilTransitionShiftEnds();
		} else if (DriverStationUtil.hasEndGameStarted().isPresent()) {
			return DriverStationUtil.timeUntilEndgameEnds();
		}
		return DriverStationUtil.timeUntilShiftEnds();
	}

	public static Optional<Double> getTimeLeftUntilActive() {
		if (isMyHubActive().isPresent()) {
			if (isMyHubActive().get()) {
				return Optional.of(0.0);
			}
		}
		return Optional.of(timeUntilCurrentShiftEndsSeconds());
	}

	public static double getTimeLeftUntilInactive() {
		if (isMyHubActive().isPresent()) {
			if (!isMyHubActive().get() || DriverStationUtil.hasGameEnded().isPresent()) {
				return 0;
			}
		}
		return timeUntilCurrentShiftEndsSeconds();
	}

	public static boolean isRobotAllianceAutoWinner() {
		return DriverStationUtil.getAlliance() == getAutoWinnerAlliance();
	}

}
