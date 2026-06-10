package frc.utils.alerts;

import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

public class Alert {

	public enum AlertType {

		ERROR,
		WARNING;

	}

	private static final String ALERT_LOG_PATH = "Alerts";

	private final String logPath;
	private final String name;
	private final boolean isDriverRelevant;

	public Alert(AlertType type, String name, boolean isDriverRelevant) {
		this.logPath = ALERT_LOG_PATH + "/" + type.toString() + "/" + name;
		this.name = name;
		this.isDriverRelevant = isDriverRelevant;
	}

	public void report() {
		Logger.recordOutput(logPath, TimeUtil.getCurrentTimeSeconds());
	}

	public String getName() {
		return name;
	}

	public boolean isDriverRelevant() {
		return isDriverRelevant;
	}

}
