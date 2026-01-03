package frc.robot.statemachine.funnelstatehandler;

public enum FunnelState {

	STOP(3, 0),
	INTAKE(),
	SHOOT(10, 0),
	SHOOT_WHILE_INTAKE(6, 0),
	DRIVE(3, 0),
	CALIBRATION;

	private final double bellyVoltage;
	private final double omniVoltage;

	FunnelState(double omniVoltage, double bellyVoltage) {
		this.bellyVoltage = bellyVoltage;
		this.omniVoltage = omniVoltage;
	}

	FunnelState(double bellyVoltage) {
		this.bellyVoltage = bellyVoltage;
		this.omniVoltage = 0;
	}

	FunnelState() {
		this.bellyVoltage = 0;
		this.omniVoltage = 0;
	}

	public double getBellyVoltage() {
		return bellyVoltage;
	}

	public double getOmniVoltage() {
		return omniVoltage;
	}

}
