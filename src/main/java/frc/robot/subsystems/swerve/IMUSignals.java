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
		Rotation2d[] rollArray = this.rollSignal.asArray();
		Rotation2d[] pitchArray = this.pitchSignal.asArray();
		Rotation2d[] yawArray = this.yawSignal.asArray();
		Rotation3d[] orientationArray = new Rotation3d[Math.min(Math.min(rollArray.length, pitchArray.length), yawArray.length)];
		for (int i = 0; i < orientationArray.length; i++) {
			orientationArray[i] = new Rotation3d(rollArray[i].getRadians(), pitchArray[i].getRadians(), yawArray[i].getRadians());
		}
		return orientationArray;
	}

	public Rotation3d getLatestOrientation() {
		return new Rotation3d(
			this.rollSignal.getLatestValue().getRadians(),
			this.pitchSignal.getLatestValue().getRadians(),
			this.yawSignal.getLatestValue().getRadians()
		);
	}

	public Translation3d[] getAllAccelerationsEarthGravitationalAcceleration() {
		Double[] xAcceleration = this.xAccelerationSignalEarthGravitationalAcceleration.asArray();
		Double[] yAcceleration = this.yAccelerationSignalEarthGravitationalAcceleration.asArray();
		Double[] zAcceleration = this.zAccelerationSignalEarthGravitationalAcceleration.asArray();
		Translation3d[] accelerationArray = new Translation3d[Math
			.min(Math.min(xAcceleration.length, yAcceleration.length), zAcceleration.length)];
		for (int i = 0; i < accelerationArray.length; i++) {
			accelerationArray[i] = new Translation3d(xAcceleration[i], yAcceleration[i], zAcceleration[i]);
		}
		return accelerationArray;
	}

	public Translation3d getLatestAccelerationEarthGravitationalAcceleration() {
		return new Translation3d(
			this.xAccelerationSignalEarthGravitationalAcceleration().getLatestValue(),
			this.yAccelerationSignalEarthGravitationalAcceleration().getLatestValue(),
			this.zAccelerationSignalEarthGravitationalAcceleration().getLatestValue()
		);
	}

}
