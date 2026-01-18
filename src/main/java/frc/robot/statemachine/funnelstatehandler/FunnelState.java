package frc.robot.statemachine.funnelstatehandler;

public enum FunnelState {

	STOP,
	INTAKE(3, 1),
	SHOOT(3, 1),
	SHOOT_WHILE_INTAKE(3, 1),
	DRIVE,
	CALIBRATION;

	private final double omniVoltage;
	private final double bellyVoltage;

	FunnelState(double omniVoltage, double bellyVoltage) {
		this.omniVoltage = omniVoltage;
		this.bellyVoltage = bellyVoltage;
	}

	FunnelState() {
		this.omniVoltage = 0;
		this.bellyVoltage = 0;
	}

	public double getOmniVoltage() {
		return omniVoltage;
	}

	public double getBellyVoltage() {
		return bellyVoltage;
	}

}
