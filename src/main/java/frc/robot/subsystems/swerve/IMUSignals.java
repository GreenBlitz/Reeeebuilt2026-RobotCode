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

	public Rotation3d getAngularVelocity() {
		return new Rotation3d(
			this.rollAngularVelocitySignal().getLatestValue().getRadians(),
			this.pitchAngularVelocitySignal().getLatestValue().getRadians(),
			this.yawAngularVelocitySignal().getLatestValue().getRadians()
		);
	}

	public Rotation3d[] getAllOrientations() {
		Rotation2d[] allRollValues = this.rollSignal.asArray();
		Rotation2d[] allPitchValues = this.pitchSignal.asArray();
		Rotation2d[] allYawValues = this.yawSignal.asArray();
		Rotation3d[] allOrientations = new Rotation3d[Math.min(Math.min(allRollValues.length, allPitchValues.length), allYawValues.length)];
		for (int i = 0; i < allOrientations.length; i++) {
			allOrientations[i] = new Rotation3d(allRollValues[i].getRadians(), allPitchValues[i].getRadians(), allYawValues[i].getRadians());
		}
		return allOrientations;
	}

	public Rotation3d getLatestOrientation() {
		return new Rotation3d(
			this.rollSignal.getLatestValue().getRadians(),
			this.pitchSignal.getLatestValue().getRadians(),
			this.yawSignal.getLatestValue().getRadians()
		);
	}

	public Translation3d[] getAllAccelerationsEarthGravitationalAcceleration() {
		Double[] allXAccelerations = this.xAccelerationSignalEarthGravitationalAcceleration.asArray();
		Double[] allYAccelerations = this.yAccelerationSignalEarthGravitationalAcceleration.asArray();
		Double[] allZAccelerations = this.zAccelerationSignalEarthGravitationalAcceleration.asArray();
		Translation3d[] allAccelerations = new Translation3d[Math
			.min(Math.min(allXAccelerations.length, allYAccelerations.length), allZAccelerations.length)];
		for (int i = 0; i < allAccelerations.length; i++) {
			allAccelerations[i] = new Translation3d(allXAccelerations[i], allYAccelerations[i], allZAccelerations[i]);
		}
		return allAccelerations;
	}

	public Translation3d getLatestAccelerationEarthGravitationalAcceleration() {
		return new Translation3d(
			this.xAccelerationSignalEarthGravitationalAcceleration().getLatestValue(),
			this.yAccelerationSignalEarthGravitationalAcceleration().getLatestValue(),
			this.zAccelerationSignalEarthGravitationalAcceleration().getLatestValue()
		);
	}

}
