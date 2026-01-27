package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.driverstation.GameSpecificMessageResponse;
import frc.utils.driverstation.DriverStationUtil;
import frc.utils.time.TimeUtil;
import frc.utils.alerts.Alert;

import java.util.Optional;

public class HubUtil {

	private static Optional<DriverStation.Alliance> autoWinnerAlliance = getAutoWinningAlliance();
	private static Optional<DriverStation.Alliance> autoLosingAlliance = getAutoLosingAlliance();

	private static Optional<DriverStation.Alliance> getAutoWinningAlliance() {
		if (!DriverStationUtil.isTeleop()) {
			return Optional.empty();
		}
		String gameData = DriverStation.getGameSpecificMessage();
		if (gameData.isEmpty()) {
			new Alert(Alert.AlertType.WARNING, "Didn't get auto winning alliance").report();
			return Optional.empty();
		}
		Optional<DriverStation.Alliance> alliance = switch (GameSpecificMessageResponse.fromChar(gameData.charAt(0))) {
			case BLUE -> Optional.of(DriverStation.Alliance.Blue);
			case RED -> Optional.of(DriverStation.Alliance.Red);
			case DEFAULT -> Optional.empty();
		};
		if (alliance.isEmpty()) {
			new Alert(Alert.AlertType.WARNING, "Unknown auto winner alliance").report();
			return Optional.empty();
		}
		return alliance;
	}

	private static Optional<DriverStation.Alliance> getAutoLosingAlliance() {
		if (autoWinnerAlliance.isEmpty()) {
			return Optional.empty();
		}
		return switch (autoWinnerAlliance.get()) {
			case Red -> Optional.of(DriverStation.Alliance.Blue);
			case Blue -> Optional.of(DriverStation.Alliance.Red);
		};
	}

	public static void refreshAlliances() {
		if (autoWinnerAlliance.isEmpty() && DriverStationUtil.isTeleop()) {
			autoWinnerAlliance = getAutoWinningAlliance();
			autoLosingAlliance = getAutoLosingAlliance();
		}
	}

	public static Optional<DriverStation.Alliance> getAutoWinnerAlliance() {
		return autoWinnerAlliance;
	}

	public static Optional<DriverStation.Alliance> getAutoLoserAlliance() {
		return autoLosingAlliance;
	}

	public static boolean isAutoWinnerShift() {
		return getShiftsPassed() % 2 != 0;
	}

	public static int getShiftsPassed() {
		if (TimeUtil.getTimeSinceTeleopInitSeconds() >= GamePeriodUtils.TELEOP_DURATION_SECONDS) {
			return (GamePeriodUtils.TELEOP_DURATION_SECONDS - GamePeriodUtils.TRANSITION_SHIFT_DURATION_SECONDS)
				/ GamePeriodUtils.ALLIANCE_SHIFT_DURATION_SECONDS;
		} else {
			return (int) (TimeUtil.getTimeSinceTeleopInitSeconds() - GamePeriodUtils.TRANSITION_SHIFT_DURATION_SECONDS)
				/ GamePeriodUtils.ALLIANCE_SHIFT_DURATION_SECONDS;
		}
	}

	public static Optional<DriverStation.Alliance> getActiveHub() {
		if (DriverStation.isAutonomous() || GamePeriodUtils.isTransitionShift() || GamePeriodUtils.isInEndgame()) {
			return Optional.of(DriverStationUtil.getAlliance());
		} else if (!DriverStationUtil.isTeleop() || GamePeriodUtils.hasGameEnded()) {
			return Optional.empty();
		}

		return isAutoWinnerShift() ? autoWinnerAlliance : autoLosingAlliance;
	}

	public static boolean isOurHubActive() {
		if (getActiveHub().isEmpty()) {
			return false;
		}
		return getActiveHub().get().equals(DriverStationUtil.getAlliance());
	}

	public static double timeUntilCurrentShiftEndsSeconds() {
		if (
			TimeUtil.getTimeSinceTeleopInitSeconds() >= GamePeriodUtils.GAME_DURATION_SECONDS || TimeUtil.getTimeSinceTeleopInitSeconds() == -1
		) {
			return 0;
		}
		if (GamePeriodUtils.isTransitionShift()) {
			return GamePeriodUtils.getTimeUntilTransitionShiftEnds();
		} else if (GamePeriodUtils.isInEndgame()) {
			return GamePeriodUtils.getTimeUntilGameEnds();
		}
		return GamePeriodUtils.getTimeUntilShiftEnds();
	}

	public static double getTimeLeftUntilActive() {
		if (isOurHubActive()) {
			return 0;
		}
		return timeUntilCurrentShiftEndsSeconds();
	}

	public static double getTimeLeftUntilInactive() {
		if (!isOurHubActive()) {
			return 0;
		}
		return timeUntilCurrentShiftEndsSeconds();
	}

	public static boolean isRobotAllianceAutoWinner() {
		if (autoWinnerAlliance.isEmpty()) {
			return false;
		}
		return DriverStationUtil.getAlliance().equals(autoWinnerAlliance.get());
	}

}
