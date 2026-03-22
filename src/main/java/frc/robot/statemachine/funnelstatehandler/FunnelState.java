package frc.robot.statemachine.funnelstatehandler;


import frc.utils.battery.BatteryUtil;

public enum FunnelState {

	STOP,
	PRE_SHOOT(7, -2),
	SHOOT( 7, 5),
	OUTTAKE(-3, -8),
	ROLL_UNTIL_SENSOR(1, 4),
	CALIBRATION;

	private final double magazineVoltage;
	private final double conveyorVoltage;

	FunnelState(double magazineVoltage, double conveyorVoltage) {
		this.magazineVoltage = magazineVoltage;
		this.conveyorVoltage = conveyorVoltage;
	}

	FunnelState() {
		this.magazineVoltage = 0;
		this.conveyorVoltage = 0;
	}

	public double getMagazineVoltage() {
		return magazineVoltage;
	}

	public double getConveyorVoltage() {
		return conveyorVoltage;
	}

}
