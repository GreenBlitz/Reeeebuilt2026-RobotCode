package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

public class OdometryData {

	private double timestampSeconds = 0;
	private SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
	private Optional<Rotation3d> imuOrientation = Optional.empty();
	private Optional<Translation2d> imuAccelerationMagnitudeG = Optional.empty();

	public OdometryData() {}

	public OdometryData(
		double timestampSeconds,
		SwerveModulePosition[] wheelPositions,
		Optional<Rotation3d> imuOrientation,
		Optional<Translation2d> imuAccelerationMagnitudeG
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

	public Optional<Translation2d> getImuAccelerationMagnitudeG() {
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
			Optional.of(
				imuOrientation.map(orientation -> new Rotation3d(imuOrientation.get().getX(), imuOrientation.get().getY(), imuYaw.getRadians()))
					.orElseGet(() -> new Rotation3d(0, 0, imuYaw.getRadians()))
			)
		);
	}

	public void setIMUAcceleration(Optional<Translation2d> imuAccelerationMagnitudeG) {
		this.imuAccelerationMagnitudeG = imuAccelerationMagnitudeG;
	}

	public void setIMUAcceleration(Translation2d imuAccelerationMagnitudeG) {
		setIMUAcceleration(Optional.of(imuAccelerationMagnitudeG));
	}

}
