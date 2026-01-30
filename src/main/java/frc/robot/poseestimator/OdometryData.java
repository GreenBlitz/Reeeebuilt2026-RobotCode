package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

public class OdometryData {

	private double timestampSeconds = 0;
	private SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
	private Optional<Rotation3d> imuOrientation = Optional.empty();
	private Optional<Double> imuAccelerationMagnitudeG = Optional.empty();

	public OdometryData() {}

	public OdometryData(
		double timestampSeconds,
		SwerveModulePosition[] wheelPositions,
		Optional<Rotation3d> imuOrientation,
		Optional<Double> imuAccelerationMagnitudeG
	) {
		this.timestampSeconds = timestampSeconds;
		this.wheelPositions = wheelPositions;
		this.imuOrientation = imuOrientation;
		this.imuAccelerationMagnitudeG = imuAccelerationMagnitudeG;
	}

	public double getTimestampSeconds() {
		return timestampSeconds;
	}

	public SwerveModulePosition[] getWheelPositions() {
		return wheelPositions;
	}

	public Optional<Rotation3d> getImuOrientation() {
		return imuOrientation;
	}

	public Optional<Double> getImuAccelerationMagnitudeG() {
		return imuAccelerationMagnitudeG;
	}

	public void setTimestamp(double timestampSeconds) {
		this.timestampSeconds = timestampSeconds;
	}

	public void setWheelPositions(SwerveModulePosition[] wheelPositions) {
		this.wheelPositions = wheelPositions;
	}

	public void setIMUOrientation(Optional<Rotation3d> imuOrientation) {
		this.imuOrientation = imuOrientation;
	}

	public void setIMUYaw(Rotation2d imuYaw) {
		setIMUOrientation(
			imuOrientation.map(orientation -> new Rotation3d(imuOrientation.get().getX(), imuOrientation.get().getY(), imuYaw.getRadians()))
		);
	}

	public void setIMUAcceleration(Optional<Double> imuAccelerationMagnitudeG) {
		this.imuAccelerationMagnitudeG = imuAccelerationMagnitudeG;
	}

	public void setIMUAcceleration(double imuAccelerationMagnitudeG) {
		setIMUAcceleration(Optional.of(imuAccelerationMagnitudeG));
	}

}
