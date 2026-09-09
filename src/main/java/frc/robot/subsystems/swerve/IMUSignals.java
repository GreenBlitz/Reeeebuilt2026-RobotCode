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
	InputSignal<Double> xAccelerationGSignal,
	InputSignal<Double> yAccelerationGSignal,
	InputSignal<Double> zAccelerationGSignal
) {

	public Rotation3d getAngularVelocity() {
		return new Rotation3d(
			rollSignal.getLatestValue().getRadians(),
			pitchSignal.getLatestValue().getRadians(),
			yawSignal.getLatestValue().getRadians()
		);
	}

	public Translation3d[] getAllAccelerationsG() {
		Double[] allXAccelerations = xAccelerationGSignal.asArray();
		Double[] allYAccelerations = yAccelerationGSignal.asArray();
		Double[] allZAccelerations = zAccelerationGSignal.asArray();
		Translation3d[] allAccelerations = new Translation3d[Math
			.min(Math.min(allXAccelerations.length, allYAccelerations.length), allZAccelerations.length)];

		for (int i = 0; i < allAccelerations.length; i++) {
			allAccelerations[i] = new Translation3d(allXAccelerations[i], allYAccelerations[i], allZAccelerations[i]);
		}
		return allAccelerations;
	}

	public Translation3d getLatestAccelerationG() {
		return new Translation3d(
			xAccelerationGSignal().getLatestValue(),
			yAccelerationGSignal().getLatestValue(),
			zAccelerationGSignal().getLatestValue()
		);
	}

}
