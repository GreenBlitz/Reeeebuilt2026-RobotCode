package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.Optional;

public class OdometryData {

	private double timestampSeconds = 0;
	private SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
	private SwerveModuleState[] wheelStates = new SwerveModuleState[4];
	private Optional<Rotation3d> imuOrientation = Optional.empty();
	private Optional<Double> imuAccelerationMagnitudeG = Optional.empty();

	public OdometryData() {}

	public OdometryData(
		double timestampSeconds,
		SwerveModulePosition[] wheelPositions,
		SwerveModuleState[] wheelStates,
		Optional<Rotation3d> imuOrientation,
		Optional<Double> imuAccelerationMagnitudeG
	) {
		this.timestampSeconds = timestampSeconds;
		this.wheelPositions = wheelPositions;
		this.wheelStates = wheelStates;
		this.imuOrientation = imuOrientation;
		this.imuAccelerationMagnitudeG = imuAccelerationMagnitudeG;
	}

	public double getTimestampSeconds() {
		return timestampSeconds;
	}

	public SwerveModulePosition[] getWheelPositions() {
		return wheelPositions;
	}

	public SwerveModuleState[] getWheelStates() {
		return wheelStates;
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

	public void setWheelStates(SwerveModuleState[] wheelStates) {
		this.wheelStates = wheelStates;
	}

	public void setIMUOrientation(Optional<Rotation3d> imuOrientation) {
		this.imuOrientation = imuOrientation;
	}

	public void setIMUOrientation(Rotation3d imuOrientation) {
		setIMUOrientation(Optional.of(imuOrientation));
	}

	public void setIMUAcceleration(Optional<Double> imuAccelerationMagnitudeG) {
		this.imuAccelerationMagnitudeG = imuAccelerationMagnitudeG;
	}

	public void setIMUAcceleration(double imuAccelerationMagnitudeG) {
		setIMUAcceleration(Optional.of(imuAccelerationMagnitudeG));
	}

}
