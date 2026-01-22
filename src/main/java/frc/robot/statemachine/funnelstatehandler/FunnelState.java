package frc.robot.statemachine.funnelstatehandler;

import edu.wpi.first.math.geometry.Rotation2d;

public enum FunnelState {

	STOP,
	INTAKE(Rotation2d.fromRotations(10), 5),
	SHOOT(Rotation2d.fromRotations(10), 5),
	SHOOT_WHILE_INTAKE(Rotation2d.fromRotations(10), 5),
	DRIVE,
	CALIBRATION;

	private final Rotation2d trainVelocity;
	private final double bellyVoltage;

	FunnelState(Rotation2d trainVelocity, double bellyVoltage) {
		this.trainVelocity = trainVelocity;
		this.bellyVoltage = bellyVoltage;
	}

	FunnelState() {
		this.trainVelocity = new Rotation2d();
		this.bellyVoltage = 0;
	}

	public Rotation2d getTrainVelocity() {
		return trainVelocity;
	}

	public double getBellyVoltage() {
		return bellyVoltage;
	}

}
