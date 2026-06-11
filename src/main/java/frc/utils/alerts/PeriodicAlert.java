package frc.utils.alerts;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

public class PeriodicAlert extends Alert {

	private final BooleanSupplier reportCondition;

	public PeriodicAlert(AlertType type, String name, BooleanSupplier reportCondition, boolean isDriverRelevant) {
		super(type, name, isDriverRelevant);
		this.reportCondition = reportCondition;
		new Trigger(reportCondition).onFalse(new InstantCommand(() -> AlertManager.getReportedAlerts().remove(this)));
	}

	public PeriodicAlert(AlertType type, String name, BooleanSupplier reportCondition) {
		super(type, name);
		this.reportCondition = reportCondition;
	}

	public void reportByCondition() {
		if (reportCondition.getAsBoolean()) {
			report();
		}
	}

}
