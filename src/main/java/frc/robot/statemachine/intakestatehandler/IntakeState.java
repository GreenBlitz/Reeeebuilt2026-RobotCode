package frc.robot.statemachine.intakestatehandler;

import edu.wpi.first.math.geometry.Rotation2d;

public enum IntakeState {

	CLOSED(Rotation2d.fromDegrees(90), 0),
	INTAKE(Rotation2d.fromDegrees(20), 0.8),
	RESET_FOUR_BAR,
	CALIBRATION,
	STAY_IN_PLACE;

	private final Rotation2d fourBarPosition;
	private final double intakePower;

	IntakeState() {
		this.fourBarPosition = Rotation2d.fromRotations(Double.NaN);
		this.intakePower = Double.NaN;
	}

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
