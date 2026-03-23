package frc.robot.statemachine.funnelstatehandler;


public enum FunnelState {

	STOP,
	PRE_SHOOT(1, -2),
	SHOOT(1, 10),
	OUTTAKE(-1, -8),
	ROLL_UNTIL_SENSOR(0.25, 4),
	CALIBRATION;

	private final double magazinePower;
	private final double conveyorVoltage;

	FunnelState(double magazineVelocity, double conveyorVoltage) {
		this.magazinePower = magazineVelocity;
		this.conveyorVoltage = conveyorVoltage;
	}

	FunnelState() {
		this.magazinePower = 0;
		this.conveyorVoltage = 0;
	}

	public double getMagazinePower() {
		return magazinePower;
	}

	public double getConveyorVoltage() {
		return conveyorVoltage;
	}

}
