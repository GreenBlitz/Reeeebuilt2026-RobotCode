package frc.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.RobotManager;
import frc.utils.time.TimeUtil;

public class DriverStationUtil {

	private static final DriverStation.Alliance DEFAULT_ALLIANCE = DriverStation.Alliance.Red;

	public static DriverStation.Alliance getAlliance() {
		return DriverStation.getAlliance().orElse(DEFAULT_ALLIANCE);
	}

	public static DriverStation.Alliance getStartingAlliance() {
		String gameData = DriverStation.getGameSpecificMessage();
		if (gameData.isEmpty()) {
			new Alert("Unknown starting alliance", Alert.AlertType.kWarning);
			return DEFAULT_ALLIANCE;
		}
		DriverStation.Alliance alliance = switch (gameData.charAt(0)) {
			case 'B' -> DriverStation.Alliance.Blue;
			case 'R' -> DriverStation.Alliance.Red;
			default -> null;
		};
		if (alliance == null) {
			new Alert("Unknown starting alliance", Alert.AlertType.kWarning);
			return DEFAULT_ALLIANCE;
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
		return DEFAULT_ALLIANCE;
	}

	public static DriverStation.Alliance whichHubIsActive() {
		if (DriverStation.isAutonomous()) {
			return DEFAULT_ALLIANCE;
		}
		if (RobotManager.getTeleopStartTime() == 0) {
			new Alert("Unkown time since teleop started", Alert.AlertType.kInfo);
			return DEFAULT_ALLIANCE;
		}
		if (DriverStation.isTeleop()) {
			int isTheCurrentShiftAShiftOfTheStartingAlliance = (((int) Math.floor(TimeUtil.getCurrentTimeSeconds())
				- (int) Math.floor(RobotManager.getTeleopStartTime())) / 25) % 2;

			DriverStation.Alliance startingAlliance = getStartingAlliance();

			return isCurrentShiftOfStartingAlliance(isTheCurrentShiftAShiftOfTheStartingAlliance)
				? startingAlliance
				: oppositeAllianceOfStartingAlliance(startingAlliance);
		}
		return DEFAULT_ALLIANCE;
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
