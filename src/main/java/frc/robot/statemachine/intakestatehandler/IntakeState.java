package frc.robot.statemachine.intakestatehandler;

import edu.wpi.first.math.geometry.Rotation2d;

public enum IntakeState {

	CLOSED(Rotation2d.fromDegrees(90), 0.3),
	INTAKE(Rotation2d.fromDegrees(22), 0.9),
	FORCE_OPEN(Rotation2d.fromDegrees(22), 0.9),
	OUTTAKE(Rotation2d.fromDegrees(22), -0.9),
	RESET_FOUR_BAR(Rotation2d.fromRotations(Double.NaN), Double.NaN),
	CALIBRATION(Rotation2d.fromRotations(Double.NaN), Double.NaN),
	STAY_IN_PLACE(Rotation2d.fromRotations(Double.NaN), Double.NaN);

	private final Rotation2d fourBarPosition;
	private final double intakePower;

	IntakeState(Rotation2d fourBarPosition, double intakePower) {
		this.fourBarPosition = fourBarPosition;
		this.intakePower = intakePower;
	}

	public Rotation2d getFourBarPosition() {
		return fourBarPosition;
	}

	public double getIntakePower() {
		return intakePower;
	}

}
