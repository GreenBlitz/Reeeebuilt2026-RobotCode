package frc.robot.statemachine.funnelstatehandler;

import edu.wpi.first.math.geometry.Rotation2d;

public enum FunnelState {

	STOP,
	SHOOT(10, 8),
	ROLL_UNTIL_SENSOR(0.1, 4),
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
