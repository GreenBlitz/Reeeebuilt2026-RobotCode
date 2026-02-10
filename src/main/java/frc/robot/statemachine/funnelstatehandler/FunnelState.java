package frc.robot.statemachine.funnelstatehandler;

import edu.wpi.first.math.geometry.Rotation2d;

public enum FunnelState {

	STOP,
	SHOOT(Rotation2d.fromRotations(18), 1),
	NEUTRAL,
	CALIBRATION;

	private final Rotation2d trainVelocity;
	private final double bellyVoltage;

	FunnelState(Rotation2d trainVelocity, double bellyVoltage) {
		this.trainVelocity = trainVelocity;
		this.bellyVoltage = bellyVoltage;
	}

	FunnelState() {
		this.trainVelocity = Rotation2d.kZero;
		this.bellyVoltage = 0;
	}

	public Rotation2d getTrainVelocity() {
		return trainVelocity;
	}

	public double getBellyVoltage() {
		return bellyVoltage;
	}

}
