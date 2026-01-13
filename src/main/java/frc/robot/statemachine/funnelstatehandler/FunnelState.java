package frc.robot.statemachine.funnelstatehandler;

public enum FunnelState {

	STOP,
	INTAKE(3),
	SHOOT(3),
	SHOOT_WHILE_INTAKE(3),
	DRIVE(),
	CALIBRATION;

	private final double omniVoltage;

	FunnelState(double omniVoltage) {
		this.omniVoltage = omniVoltage;
	}

	FunnelState() {
		this.omniVoltage = 0;
	}

	public double getOmniVoltage() {
		return omniVoltage;
	}

}
