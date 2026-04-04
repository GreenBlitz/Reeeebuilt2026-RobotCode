package frc.robot.statemachine.funnelstatehandler;


import edu.wpi.first.math.geometry.Rotation2d;

public enum FunnelState {

	STOP,
	PRE_SHOOT(Rotation2d.fromRotations(70), -2),
	SHOOT(Rotation2d.fromRotations(70), 8.4),
	OUTTAKE(Rotation2d.fromRotations(Double.NaN), -8),
	ROLL_UNTIL_SENSOR(Rotation2d.fromRotations(70), 4),
	OUTTAKE_SHOOT(Rotation2d.fromRotations(70), -12),
	CALIBRATION;

	private final Rotation2d magazineVelocity;
	private final double conveyorVoltage;

	FunnelState(Rotation2d magazineVelocity, double conveyorVoltage) {
		this.magazineVelocity = magazineVelocity;
		this.conveyorVoltage = conveyorVoltage;
	}

	FunnelState() {
		this.magazineVelocity = Rotation2d.kZero;
		this.conveyorVoltage = 0;
	}

	public Rotation2d getMagazineVelocity() {
		return magazineVelocity;
	}

	public double getConveyorVoltage() {
		return conveyorVoltage;
	}

}
