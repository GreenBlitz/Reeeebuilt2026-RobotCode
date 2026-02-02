package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.*;
import frc.robot.hardware.interfaces.InputSignal;

public record IMUSignals(
	InputSignal<Rotation2d> rollSignal,
	InputSignal<Rotation2d> pitchSignal,
	InputSignal<Rotation2d> yawSignal,
	InputSignal<Rotation2d> rollAngularVelocitySignal,
	InputSignal<Rotation2d> pitchAngularVelocitySignal,
	InputSignal<Rotation2d> yawAngularVelocitySignal,
	InputSignal<Double> xAccelerationSignalEarthGravitationalAcceleration,
	InputSignal<Double> yAccelerationSignalEarthGravitationalAcceleration,
	InputSignal<Double> zAccelerationSignalEarthGravitationalAcceleration
) {

	public Rotation2d[] getAngularVelocity() {
		return new Rotation2d[] {
			rollAngularVelocitySignal.getLatestValue(),
			pitchAngularVelocitySignal.getLatestValue(),
			yawAngularVelocitySignal.getLatestValue()};
	}

	public Rotation3d getOrientation() {
		return new Rotation3d(
			rollSignal.getLatestValue().getRadians(),
			pitchSignal.getLatestValue().getRadians(),
			yawSignal.getLatestValue().getRadians()
		);
	}

	public Translation3d getAccelerationEarthGravitationalAcceleration() {
		return new Translation3d(
			xAccelerationSignalEarthGravitationalAcceleration.getLatestValue(),
			yAccelerationSignalEarthGravitationalAcceleration.getLatestValue(),
			zAccelerationSignalEarthGravitationalAcceleration.getLatestValue()
		);
	}

}
