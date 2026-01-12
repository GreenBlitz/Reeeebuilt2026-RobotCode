package frc.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;

public class DriverStationUtil {

	private static final DriverStation.Alliance DEFAULT_ALLIANCE = DriverStation.Alliance.Red;

	public static DriverStation.Alliance getAlliance() {
		return DriverStation.getAlliance().orElse(DEFAULT_ALLIANCE);
	}

	public static DriverStation.Alliance getAutoWinnerAlliance() {
		String gameData = DriverStation.getGameSpecificMessage();
		if (gameData.isEmpty()) {
			new Alert("Unknown current active alliance", Alert.AlertType.kWarning);
			return DEFAULT_ALLIANCE;
		}
		DriverStation.Alliance alliance = switch (gameData.charAt(0)) {
			case 'B' -> DriverStation.Alliance.Blue;
			case 'R' -> DriverStation.Alliance.Red;
			default -> null;
		};
		if (alliance == null) {
			new Alert("Unknown current active alliance", Alert.AlertType.kWarning);
			return DEFAULT_ALLIANCE;
		}
		return alliance;
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

}
