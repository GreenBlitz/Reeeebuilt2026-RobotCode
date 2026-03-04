package frc.robot.statemachine.funnelstatehandler;

import edu.wpi.first.math.geometry.Rotation2d;

public enum FunnelState {

	STOP,
	PRE_SHOOT(Rotation2d.fromRotations(40), -2),
	SHOOT(Rotation2d.fromRotations(40), 8),
	OUTTAKE(Rotation2d.fromRotations(Double.NaN), -8),
	ROLL_UNTIL_SENSOR(Rotation2d.fromRotations(18), 4),
	CALIBRATION;

	private final Rotation2d trainVelocity;
	private final double conveyorVoltage;

	FunnelState(Rotation2d trainVelocity, double conveyorVoltage) {
		this.trainVelocity = trainVelocity;
		this.conveyorVoltage = conveyorVoltage;
	}

	FunnelState() {
		this.trainVelocity = Rotation2d.kZero;
		this.conveyorVoltage = 0;
	}

	public Rotation2d getTrainVelocity() {
		return trainVelocity;
	}

	public double getConveyorVoltage() {
		return conveyorVoltage;
	}

}
