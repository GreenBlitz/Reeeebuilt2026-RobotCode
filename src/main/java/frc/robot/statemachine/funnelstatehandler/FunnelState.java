package frc.robot.statemachine.funnelstatehandler;

public enum FunnelState {

	STOP,
	INTAKE(3, 1),
	SHOOT(3, 1),
	SHOOT_WHILE_INTAKE(3, 1),
	DRIVE,
	CALIBRATION;

	private final double trainVoltage;
	private final double bellyVoltage;

	FunnelState(double trainVoltage, double bellyVoltage) {
		this.trainVoltage = trainVoltage;
		this.bellyVoltage = bellyVoltage;
	}

	FunnelState() {
		this.trainVoltage = 0;
		this.bellyVoltage = 0;
	}

	public double getTrainVoltage() {
		return trainVoltage;
	}

	public double getBellyVoltage() {
		return bellyVoltage;
	}

}
