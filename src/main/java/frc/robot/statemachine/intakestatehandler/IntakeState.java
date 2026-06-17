package frc.robot.statemachine.intakestatehandler;

import edu.wpi.first.math.geometry.Rotation2d;

public enum IntakeState {

	CLOSED(Rotation2d.fromDegrees(75), 0.3),
	SLOW_CLOSE(Rotation2d.fromDegrees(Double.NaN), 0.3),
	INTAKE(Rotation2d.fromDegrees(18.35), 0.9),
	OUTTAKE(Rotation2d.fromDegrees(18.35), -0.9),
	RESET_PIVOT(Rotation2d.fromRotations(Double.NaN), Double.NaN),
	CALIBRATION(Rotation2d.fromRotations(Double.NaN), Double.NaN),
	STAY_IN_PLACE(Rotation2d.fromRotations(Double.NaN), Double.NaN);

	private final Rotation2d pivotPosition;
	private final double intakePower;

	IntakeState(Rotation2d pivotPosition, double intakePower) {
		this.pivotPosition = pivotPosition;
		this.intakePower = intakePower;
	}

	public Rotation2d getPivotPosition() {
		return pivotPosition;
	}

	public double getIntakePower() {
		return intakePower;
	}

}
