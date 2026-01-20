package frc.utils.driverstation;

import edu.wpi.first.wpilibj.DriverStation;
import frc.constants.GamePeriodConstants;
import frc.utils.HubUtil;

import java.util.Optional;

public class DriverStationUtil {

	static final DriverStation.Alliance DEFAULT_ALLIANCE = DriverStation.Alliance.Red;

	public static Optional<DriverStation.Alliance> getAlliance() {
		return DriverStation.getAlliance();
	}

	public static boolean isBlueAlliance() {
		return getAlliance().equals(DriverStation.Alliance.Blue);
	}

	public static boolean isRedAlliance() {
		return !isBlueAlliance();
	}

	public static boolean isConnectedToFMS() {
		return DriverStation.isFMSAttached();
	}

	public static boolean isAutonomous() {
		return DriverStation.isAutonomous();
	}

	public static boolean isAutonomousEnabled() {
		return DriverStation.isAutonomousEnabled();
	}

	public static boolean isTeleop() {
		return DriverStation.isTeleop();
	}

	public static boolean isTeleopEnabled() {
		return DriverStation.isTeleopEnabled();
	}

	public static boolean isTest() {
		return DriverStation.isTest();
	}

	public static boolean isTestEnabled() {
		return DriverStation.isTestEnabled();
	}

	public static boolean isDisabled() {
		return DriverStation.isDisabled();
	}

	public static boolean isMatch() {
		return DriverStation.getMatchType() != DriverStation.MatchType.None;
	}

	public static Optional<DriverStation.Alliance> isTransitionShift() {
		if (HubUtil.getTimeSinceTeleopInitSeconds() <= GamePeriodConstants.TRANSITION_SHIFT_TIME_SECONDS) {
			return DriverStationUtil.getAlliance();
		}
		return Optional.empty();
	}

	public static Optional<Boolean> hasGameEnded() {
		if (HubUtil.getTimeSinceTeleopInitSeconds() >= GamePeriodConstants.GAME_END_TIME_SECONDS) {
			return Optional.of(true);
		}
		return Optional.empty();
	}

	public static Optional<DriverStation.Alliance> hasEndGameStarted() {
		if (HubUtil.getTimeSinceTeleopInitSeconds() >= GamePeriodConstants.ENDGAME_START_TIME_SECONDS_SINCE_TELEOP) {
			return DriverStationUtil.getAlliance();
		}
		return Optional.empty();
	}

	public static Optional<Boolean> isNotTeleop() {
		if (!DriverStation.isTeleop()) {
			return Optional.of(true);
		}
		return Optional.empty();
	}

	public static Optional<DriverStation.Alliance> isAuto() {
		if (DriverStation.isAutonomous()) {
			return DriverStationUtil.getAlliance();
		}
		return Optional.empty();
	}

	public static double timeUntilTransitionShiftEnds() {
		if (DriverStationUtil.isTransitionShift().isPresent()) {
			return GamePeriodConstants.TRANSITION_SHIFT_TIME_SECONDS - HubUtil.getTimeSinceTeleopInitSeconds();
		}
		return -1;
	}

	public static double timeUntilEndgameEnds() {
		if (DriverStationUtil.hasEndGameStarted().isPresent()) {
			return GamePeriodConstants.ENDGAME_LENGTH_SECONDS
				- (HubUtil.getTimeSinceTeleopInitSeconds() - GamePeriodConstants.ENDGAME_START_TIME_SECONDS_SINCE_TELEOP);
		}
		return -1;
	}

	public static double timeUntilShiftEnds() {
		if (DriverStationUtil.isNotTeleop().isEmpty()) {
			return GamePeriodConstants.ALLIANCE_SHIFT_LENGTH_SECONDS
				- ((HubUtil.getTimeSinceTeleopInitSeconds() - GamePeriodConstants.TRANSITION_SHIFT_TIME_SECONDS)
					% GamePeriodConstants.ALLIANCE_SHIFT_LENGTH_SECONDS);
		}
		return -1;
	}

}
